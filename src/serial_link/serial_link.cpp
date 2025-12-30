#include <utility>
#include <cassert>
#include <cerrno>
#include <format>
#include <functional>
#include <memory>
#include <sys/select.h>
#include <chrono>
#include <logger.h>
#include "serial_link.h"
#include "serial_settings.h"

#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'

serial_bridge::SerialLink::SerialLink(const std::string& dev, int instance_id )
    : m_rfds({0}), m_wfds({0}),
    m_xfds({0}),tv({0,0}),
    m_read_eagained_flag(false)
{
    m_serial_fd = -1;
    m_instance_id = instance_id;
#if 0
    if (dev == "") {
        while (1) {
            RBL_LOG_FMT("looking for serial device")
            auto port_path_list = list_serial_devices();
            if (port_path_list.size() != 1) {
                throw std::runtime_error("could not find exactly one suitable serial port path");
            }
        }
    }
#endif
    m_device = std::string{dev};
    int fd = open_serial_non_blocking(dev);
    if (fd < 0) {
        throw std::runtime_error(std::format("Failed to open serial device {}", dev));
    }
    m_serial_fd = fd;
    m_read_eagained_flag = false;
    RBL_LOG_FMT("succeeded in openning %s fd: %d ", dev.c_str(), m_serial_fd)
    FD_ZERO(&m_rfds);
    FD_ZERO(&m_wfds);
    FD_ZERO(&m_xfds);
    /**
     * Setup a guard condition to be triggered by the communication
     * thread when it has a message for the node to process.
     * Thhis all happens inside threadsafe::TriggerQueue()
    */
    m_recv_callback = nullptr;
    m_output_queue_uptr = std::make_unique<threadsafe::FdQueue<IoBuffer::UPtr>>();
    m_output_queue_fd = m_output_queue_uptr->read_fileno();
    m_nbr_fds = std::max(m_serial_fd, m_output_queue_fd) + 1;
    m_read_buffer_uptr = std::make_unique<IoBuffer>();
    m_write_buffer_uptr = nullptr;
    apply_default_settings(fd);
}

serial_bridge::SerialLink::~SerialLink()
{
    close(m_serial_fd);
}
void serial_bridge::SerialLink::send_threadsafe(IoBuffer::UPtr buffer_uptr) const
{
    m_output_queue_uptr->put(std::move(buffer_uptr));
}
void serial_bridge::SerialLink::run(OnRecvCallback recv_msg_cb)
{
    m_recv_callback = std::move(recv_msg_cb);
    FD_SET(m_serial_fd, &m_rfds);
    FD_SET(m_serial_fd, &m_xfds);
    FD_SET(m_output_queue_fd, &m_rfds);
    FD_SET(m_output_queue_fd, &m_xfds);
    m_read_eagained_flag = true;
    try {
        while (true) {
            FD_SET(m_serial_fd, &m_rfds);
            FD_SET(m_output_queue_fd, &m_rfds);
            m_nbr_fds = std::max(m_serial_fd, m_output_queue_fd) + 1;
            // if (m_instance_id == 2) {
            //     std::cout << "instance_id: " << m_instance_id <<" Before select m_output_queue_fd: " << m_output_queue_fd <<  FD_ISSET(m_output_queue_fd, &m_rfds) << "\n";
            //     std::cout << "instance_id: " << m_instance_id <<" Before select rd m_serial_fd:       " << m_serial_fd <<  FD_ISSET(m_serial_fd, &m_rfds) << "\n";
            //     std::cout << "instance_id: " << m_instance_id <<" Before select wr m_serial_fd:       " << m_serial_fd <<  FD_ISSET(m_serial_fd, &m_wfds) << "\n";
            //     std::cout << "outputqueue_count: " << m_output_queue_uptr->count() << std::endl;
            // }
            int retval = select(m_nbr_fds, &m_rfds, &m_wfds, &m_xfds, nullptr);

            if (retval < 0) {
                int saved_errno = errno;
                throw std::runtime_error(
                        std::format("select error retval: %d  errno: %d  strerror: %s", retval, saved_errno,
                                    strerror(saved_errno)));
            } else if (retval == 0) {
                throw std::runtime_error("select returned zero but no timeout was set");
            } else {
                // if (m_instance_id == 2) {
                //     std::cout << "m_instance_id" << m_instance_id << "queue.count " << m_output_queue_uptr->count()  <<  std::endl;
                // }
                if(FD_ISSET(m_output_queue_fd, &m_rfds) || (!m_output_queue_uptr->empty())) {
                    // std::cout << "m_output_queue_fd ready to read instance:" << m_instance_id << std::endl;
                    RBL_LOG_FMT("serial_link::about to try write FD_ISSET");
                    this->try_write();
                }
                if(m_write_buffer_uptr) {
                    RBL_LOG_FMT("serial_link::about to try write m_write_buffer_uptr != nullptr");
                    this->try_write();
                }
                if (FD_ISSET(m_serial_fd, &m_rfds)) {
                    RBL_LOG_FMT("try_read - select says there is data");
                    m_read_eagained_flag = false;
                    this->try_read();
                } else {
                    RBL_LOG_FMT("NOT try_read - FD_ISSET false");
                }
            }
        }
    } catch (std:: exception& e) {
        RBL_LOG_FMT("serial_link caught an exception  what: %s\n", e.what());
    }
}
///
///  m_write_buffer_uptr is a state variable
///
///  This function is called whenever m_serial_fd is ready for write.
///
///  If m_write_buffer_uptr == nullptr then no write is in process and need to check the write_queue to get a buffer
///  to write.
///
///  else if m_write_buffer_uptr != nullptr a write is already underway and just have to perform a write() call
///
void serial_bridge::SerialLink::try_write()
{
    FD_CLR(m_serial_fd, &m_wfds);
    if(m_write_buffer_uptr == nullptr) {
        RBL_LOG_FMT("try_write getting from output queue");
        auto result = m_output_queue_uptr->get_nowait();
        if (!result) {
            return;
        }
        m_write_buffer_uptr = std::move(result.value());
        // std::cout << "try_write:" << m_instance_id << "from queue " << m_write_buffer_uptr->c_str() << "\n";
        RBL_LOG_FMT("try_write got this from output queue %s", m_write_buffer_uptr->to_string().c_str());
    }
    assert(m_write_buffer_uptr != nullptr);
    if (m_write_buffer_uptr->data_len() == 0)
        assert(m_write_buffer_uptr->data_len() > 0);
    RBL_LOG_FMT("try_write output buffer %s", m_write_buffer_uptr->to_string().c_str());
    const ssize_t n = write(m_serial_fd, m_write_buffer_uptr->data(), m_write_buffer_uptr->size());
    RBL_LOG_FMT("try_write after write n: %ld", n);
    int saved_write_errno = errno;
    // std::cout << "instance_id: " << m_instance_id << "write n: " << n << std::endl;
    if(n > 0 && n == static_cast<ssize_t>(m_write_buffer_uptr->size())) {
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
///
/// This function is called when select determines that there is data to read. It reads once either a buffer full or
/// as much as is available (whichever is smaller) and parses the protocol frame into a buffer holding the "message".
/// The message should be parsed by a higher level.
///
void serial_bridge::SerialLink::try_read() 
{
    RBL_LOG_FMT("try_read called \n");

    ///
    /// assert invariant m_read_buffer_uptr should never be nullptr and the buffer should be empty at this point
    ///
    assert((m_read_buffer_uptr != nullptr) && (m_read_buffer_uptr->empty()));
    const ssize_t n = read(m_serial_fd, m_read_buffer_uptr->space_ptr(), m_read_buffer_uptr->space_len());
    int saved_read_errno = errno;
    if(n == 0) {
        throw std::runtime_error(std::format("read returned zero errno: %d msg: %s\n", saved_read_errno, strerror(saved_read_errno)));
    } else if ((n < 0) && (saved_read_errno != EAGAIN)) {
        throw std::runtime_error(std::format("read returned -ve and no EAGAIN  errno: %d msg: %s\n", saved_read_errno, strerror(saved_read_errno)));
    } else if(n < 0 && saved_read_errno == EAGAIN) {
        m_read_eagained_flag = true;
        // FD_SET(m_serial_fd, &m_rfds);
    } else if(static_cast<std::size_t>(n) > m_read_buffer_uptr->space_len()) {
        throw std::runtime_error("no room in buffer to commit - logic error");
    } else {
        m_read_buffer_uptr->commit(n);
        RBL_LOG_FMT("try_read got %s\n", m_read_buffer_uptr->to_string().c_str());
    }
    // }
    if(m_read_buffer_uptr && (! m_read_buffer_uptr->empty())) {
        m_parser.consume(*m_read_buffer_uptr, [this](IoBuffer::UPtr up)
        {
            assert(up != nullptr);
            this->m_recv_callback(std::move(up));
        });
    }
    ///
    /// assert invariant - m_read_buffer_uptr is never nullptr and all data from read should be consumed by this point
    ///
    assert((m_read_buffer_uptr != nullptr) && (m_read_buffer_uptr->empty()));
}
