#include <utility>
#include <cstdio>
#include <cerrno>
#include <format>
#include <sys/select.h>
#include <poll.h>
#include "SerialPort.h"
#include "ros2_bridge.h"
#include "msgs.h"
#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'

#if 0
struct ros2_bridge::Bridge::Impl {
    LibSerial::SerialPort m_serial_port;
    threadsafe::FdQueue<std::string> m_output_queue;
    int m_serial_fd;
    int m_output_queue_fd;
    fd_set m_rfds;
    fd_set m_wfds;
    fd_set m_xfds;
    int    m_nbr_fds;
    struct timeval  tv;
    IoBuffer m_write_buffer;        //holds the ddata bytes that are currenty being written
    IoBuffer m_read_buffer;         // holds the databytes that have been read but not processed into messages
    IoBuffer m_input_message_buffer;// holds the bytes that have been processd from m_read_buffer into the
                                    // currently accumulating raw message packet
    void try_write();
    void try_read();
    Impl(LibSerial::SerialPort port) {
        m_serial_port = std::move(port);
        m_serial_fd = m_serial_port.GetFileDescriptor();
        FD_ZERO(&m_rfds);
        FD_ZERO(&m_wfds);
        FD_ZERO(&m_xfds);
        m_output_queue_fd = m_output_queue.read_fileno();
        m_nbr_fds = std::max(m_serial_fd, m_output_queue_fd) + 1;
    }
};
#endif

ros2_bridge::Bridge::Bridge(
    int serial_fd,
    threadsafe::TriggerQueue<Message> & receive_queue,
    threadsafe::FdQueue<Message>      & send_queue 
)
:m_client_queue(receive_queue), m_output_queue(send_queue)
{
    m_serial_fd = serial_fd;

    FD_ZERO(&m_rfds);
    FD_ZERO(&m_wfds);
    FD_ZERO(&m_xfds);
    m_output_queue_fd = m_output_queue.read_fileno();
    m_nbr_fds = std::max(m_serial_fd, m_output_queue_fd) + 1;
}

void ros2_bridge::Bridge::send_threadsafe(Message msg)
{
    m_output_queue.put(msg);
}
void ros2_bridge::Bridge::run() 
{
    int             retval;
    FD_SET(m_serial_fd, &m_rfds);
    FD_SET(m_output_queue_fd, &m_rfds);

    while(1) {
        retval = select(m_nbr_fds, &m_rfds, &m_wfds, &m_xfds, nullptr);
        if (retval < 0) {
            int saved_errno = errno;
            printf("select returned -1 errno is %d, %s", errno, strerror(errno));
            throw std::runtime_error(
                    std::format("select error retval: %d  errno: %d  strerror: %s", retval, saved_errno,
                                strerror(saved_errno)));
        } else if (retval == 0) {
            throw std::runtime_error("select returned zero but no timeout was set");
        } else {
            this->try_write();
            if (FD_ISSET(m_serial_fd, &m_rfds)) {
                this->try_read();
            }
        }
    }
}
void ros2_bridge::Bridge::try_write() 
{
    Message output_msg;
    FD_CLR(m_serial_fd, &m_wfds);
    if(m_write_buffer.empty()) {
        if(!m_output_queue.empty()) {
            m_output_queue.get(output_msg);
            serializer(output_msg, m_write_buffer);
        }
    }
    if(! m_write_buffer.empty()) {
        ssize_t n = write(m_serial_fd, m_write_buffer.data(), m_write_buffer.size());
        int saved_write_errno = errno;
        if(n > 0 && n == (ssize_t)m_write_buffer.size()) {
            m_write_buffer.consume(n);
        } else if (n > 0) {
            m_write_buffer.consume(n);
        } else if(n == 0) {
            throw std::runtime_error(std::format("write returned zero errno: %d msg: %s\n", saved_write_errno, strerror(saved_write_errno)));
        } else if(saved_write_errno == EAGAIN) {
                FD_SET(m_serial_fd, &m_wfds);
        } else {
            throw std::runtime_error(std::format("write returned -1 errno: %d msg: %s\n", saved_write_errno, strerror(saved_write_errno)));
        }
    }
}
void ros2_bridge::Bridge::try_read() 
{
    IoBuffer read_buffer{};
    if(m_read_buffer.empty()) {
        // only do more reading when the read buffer has been processed to empty
        // auto b = m_read_buffer.space_ptr();
        // auto len = m_read_buffer.space_len();
        ssize_t n = read(m_serial_fd, m_read_buffer.space_ptr(), m_read_buffer.space_len());
        int saved_read_errno = errno;
        if(n == 0) {
            throw std::runtime_error(std::format("read returned zero errno: %d msg: %s\n", saved_read_errno, strerror(saved_read_errno)));
        } else if(n < 0 && saved_read_errno == EAGAIN) {
            ; //no data available to read
        } else if(n < 0) {
            throw std::runtime_error(std::format("read returned -ve and no EAGAIN  errno: %d msg: %s\n", saved_read_errno, strerror(saved_read_errno)));
        } else {// n > 0 we read some data so commit it
            m_read_buffer.commit(n);
        }
    }

    if(! m_read_buffer.empty()) {
        char* p;
        while((! m_read_buffer.empty()) && (p = m_read_buffer.get_first_char_ptr()) != nullptr) {
            char ch = *p;
            m_read_buffer.consume(1);
            if(ch == CARRIAGE_RETURN) {
                ; // ignore it
            } else if(ch == LINE_FEED) {
                // end of a message
                Message input_message = deserializer(m_input_message_buffer);
                m_input_message_buffer.clear();
                m_client_queue.put(std::move(input_message));
                // keep going
            } else {
                m_input_message_buffer.append(&ch, 1);
            }
        }
    }
}
