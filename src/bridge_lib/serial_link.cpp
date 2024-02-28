#include <utility>
#include <cstdio>
#include <cerrno>
#include <format>
#include <functional>
#include <memory>
#include <sys/select.h>
#include <poll.h>
#include "rclcpp/rclcpp.hpp"

#include "SerialPort.h"
#include "serial_link.h"
#include "msgs.h"
#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'

ros2_bridge::SerialLink::SerialLink(
    int serial_fd,
    rclcpp::Node* companion_node_ptr,
    OnRecvCallback on_recv_callback
): m_companion_node_ptr(companion_node_ptr)
{
    m_serial_fd = serial_fd;

    FD_ZERO(&m_rfds);
    FD_ZERO(&m_wfds);
    FD_ZERO(&m_xfds);
    /**
     * Setup a guard condition to be triggered by the communication
     * thread when it has a message for the node to process.
     * Thhis all happens inside threadsafe::TriggerQueue() 
    */
    auto context            = m_companion_node_ptr->get_node_base_interface()->get_context();
    m_guard_condition_sptr  = std::make_shared<rclcpp::GuardCondition>(context);
    auto f                  = std::bind(&SerialLink::guard_condition_callback, this, std::placeholders::_1);
    m_guard_condition_sptr->set_on_trigger_callback(f);

    m_recv_callback = on_recv_callback;
    m_client_queue_uptr = std::make_unique<threadsafe::TriggerQueue<IoBuffer::UPtr>>(std::bind(&SerialLink::guard_trigger_function, this));
    m_output_queue_uptr = std::make_unique<threadsafe::FdQueue<IoBuffer::UPtr>>();
    m_output_queue_fd = m_output_queue_uptr->read_fileno();
    m_nbr_fds = std::max(m_serial_fd, m_output_queue_fd) + 1;
    m_read_buffer_uptr  = nullptr;
    m_write_buffer_uptr = nullptr;
}
ros2_bridge::SerialLink::~SerialLink()
{
    close(m_serial_fd);
}
void ros2_bridge::SerialLink::guard_trigger_function()
{
    m_guard_condition_sptr->trigger();
}
void ros2_bridge::SerialLink::guard_condition_callback([[maybe_unused]]std::size_t n)
{
    IoBuffer::UPtr input_buffer_uptr  = std::make_unique<IoBuffer>();
    bool gotone = m_client_queue_uptr->get_nowait(input_buffer_uptr);
    if(gotone) {
        m_recv_callback(std::move(input_buffer_uptr));
    }
    return;
}
void ros2_bridge::SerialLink::send_threadsafe(IoBuffer::UPtr buffer_uptr)
{
    // printf("send_threadsafe msg: %s\n", msg.c_str());
    m_output_queue_uptr->put(std::move(buffer_uptr));
}
void ros2_bridge::SerialLink::start_thread()
{
    std::jthread t{[this](){
        sleep(2);
        run();
    }};
}
rclcpp::Logger ros2_bridge::SerialLink::get_logger()
{
    return m_companion_node_ptr->get_logger();
}
void ros2_bridge::SerialLink::run() 
{
    int             retval;
    FD_SET(m_serial_fd, &m_rfds);
    FD_SET(m_output_queue_fd, &m_rfds);

    RCLCPP_INFO(this->get_logger(), "serial_link starting up");

    while(1) {
        // printf("about to select m_nbr_fds: %d serial_fd:%d output_queue_fd: %d \n", m_nbr_fds, m_serial_fd, m_output_queue_fd);
        retval = select(m_nbr_fds, &m_rfds, &m_wfds, &m_xfds, nullptr);
        // printf("after select retval %d \n", retval);
        // printf("FD_SET(serial_fd, m_rfds): %d FD_SET(q_fd, m_rfds): %d\n", FD_ISSET(m_serial_fd, &m_rfds), FD_ISSET(m_output_queue_fd, &m_rfds));
        if (retval < 0) {
            int saved_errno = errno;
            // printf("select returned -1 errno is %d, %s", errno, strerror(errno));
            throw std::runtime_error(
                    std::format("select error retval: %d  errno: %d  strerror: %s", retval, saved_errno,
                                strerror(saved_errno)));
        } else if (retval == 0) {
            throw std::runtime_error("select returned zero but no timeout was set");
        } else {
            // printf("serial_link::run about to try write\n");
            this->try_write();
            if (FD_ISSET(m_serial_fd, &m_rfds)) {
                this->try_read();
            }
        }
    }
}
/**
 * The next 2 functins need a serious cleanup - maybe present the logic as nested FSM
*/
void ros2_bridge::SerialLink::try_write() 
{
    FD_CLR(m_serial_fd, &m_wfds);
    if(m_write_buffer_uptr == nullptr) {
        // printf("try_write getting from output queue\n");
        [[maybe_unused]]bool gotone = m_output_queue_uptr->get_nowait(m_write_buffer_uptr);
        if(gotone) {
            // printf("try_write got this from output queue %s\n", m_write_buffer_uptr->to_string().c_str());
        } else {
            return;
        }
    }
    if(m_write_buffer_uptr) {
        // printf("try_write output buffer %s\n", m_write_buffer.to_string().c_str());
        ssize_t n = write(m_serial_fd, m_write_buffer_uptr->data(), m_write_buffer_uptr->size());
        // printf("try_write after write n: %ld\n", n);
        int saved_write_errno = errno;
        if(n > 0 && n == (ssize_t)m_write_buffer_uptr->size()) {
            m_write_buffer_uptr->consume(n);
            m_write_buffer_uptr = nullptr; // written the entire buffer - throw it away
        } else if (n > 0) {
            m_write_buffer_uptr->consume(n);
        } else if(n == 0) {
            throw std::runtime_error(std::format("write returned zero errno: %d msg: %s\n", saved_write_errno, strerror(saved_write_errno)));
        } else if(saved_write_errno == EAGAIN) {
                FD_SET(m_serial_fd, &m_wfds);
        } else {
            throw std::runtime_error(std::format("write returned -1 errno: %d msg: %s\n", saved_write_errno, strerror(saved_write_errno)));
        }
    }
}
void ros2_bridge::SerialLink::try_read() 
{
    if((m_read_buffer_uptr != nullptr) && (!m_read_buffer_uptr->empty())) {
        // printf("m_read_buffer_uptr is not null and not empty\n");
        return;
    }
    m_read_buffer_uptr = std::make_unique<IoBuffer>();
    if(m_read_buffer_uptr->empty()) {
        // only do more reading when the read buffer has been processed to empty
        // auto b = m_read_buffer.space_ptr();
        if(m_read_buffer_uptr->space_len() == 0) {
            RCLCPP_WARN(this->get_logger(), "m_read_buffer space_len == 0");
        }
        ssize_t n = read(m_serial_fd, m_read_buffer_uptr->space_ptr(), m_read_buffer_uptr->space_len());
        int saved_read_errno = errno;
        if(n == 0) {
            throw std::runtime_error(std::format("read returned zero errno: %d msg: %s\n", saved_read_errno, strerror(saved_read_errno)));
        } else if(n < 0 && saved_read_errno == EAGAIN) {
            ; //no data available to read
        } else if(n < 0) {
            throw std::runtime_error(std::format("read returned -ve and no EAGAIN  errno: %d msg: %s\n", saved_read_errno, strerror(saved_read_errno)));
        } else {// n > 0 we read some data so commit it
            if(m_read_buffer_uptr->size() + n > m_read_buffer_uptr->capacity()) {
                RCLCPP_WARN(this->get_logger(), "m_read_buffer size == capacity ");
            }
            m_read_buffer_uptr->commit(n);
            // printf("try_read got %s\n", m_read_buffer_uptr->to_string().c_str());
        }
    }

    if(m_read_buffer_uptr && (! m_read_buffer_uptr->empty())) {
        if(!m_input_message_buffer_uptr)
            m_input_message_buffer_uptr = std::make_unique<IoBuffer>();
        char* p;
        while((! m_read_buffer_uptr->empty()) && (p = m_read_buffer_uptr->get_first_char_ptr()) != nullptr) {
            char ch = *p;
            m_read_buffer_uptr->consume(1);
            if(ch == CARRIAGE_RETURN) {
                RCLCPP_WARN(this->get_logger(), "got a CR character - these should not be present in the LINE protocol.");
            } else if(ch == LINE_FEED) {
                if(m_input_message_buffer_uptr->empty()) {
                    // RCLCPP_WARN(this->get_logger(), "We got a packet of no bytes ");
                } else {
                    // printf("try_read got a message %s\n", m_input_message_buffer_uptr->to_string().c_str());
                    std::size_t qn = m_client_queue_uptr->put(std::move(m_input_message_buffer_uptr));
                    m_input_message_buffer_uptr = std::make_unique<IoBuffer>();

                    if(qn > 5) {
                        RCLCPP_WARN(this->get_logger(), "Trigger Queue has %ld entries ", qn);
                        /**
                        * @TODO - may have to trottle the reader is the Trigger Queue gets too large
                        */
                    }
                }
            } else {
                m_input_message_buffer_uptr->append(&ch, 1);
            }
        }
    }
}
