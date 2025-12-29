#include <utility>
#include <cassert>
#include <cerrno>
#include <format>
#include <functional>
#include <memory>
#include <sys/select.h>
#include <chrono>
#define RBL_LOG_ENABLED
#define RBL_LOG_ALLOW_GLOBAL
#include <logger.h>
#include "serial_link.h"
#include "serial_settings.h"

#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'

serial_bridge::SerialLink::LineParser::LineParser()
{
    m_input_message_buffer_uptr = std::make_unique<IoBuffer>();
}
void serial_bridge::SerialLink::LineParser::consume(rbl::IoBuffer& iob, const std::function<void(IoBuffer::UPtr)>& new_msg_cb)
{
    char* p;
    while((! iob.empty()) && (p = iob.get_first_char_ptr()) != nullptr) {
        char ch = *p;
        iob.consume(1);
        if(ch == CARRIAGE_RETURN) {
            printf("got a CR character - these should not be present in the LINE protocol.");
        } else if(ch == LINE_FEED) {
            if(iob.empty()) {
                // RCLCPP_WARN(this->get_logger(), "We got a packet of no bytes ");
            } else {
                new_msg_cb(std::move(m_input_message_buffer_uptr));
                m_input_message_buffer_uptr = nullptr;
                //                    std::size_t qn = m_client_queue_uptr->put(std::move(m_input_message_buffer_uptr));
                m_input_message_buffer_uptr = std::make_unique<IoBuffer>();
            }
        } else {
            m_input_message_buffer_uptr->append(&ch, 1);
        }
    }
}

serial_bridge::SerialLink::SerialLink(const std::string& dev )
    : m_rfds({0}), m_wfds({0}),
    m_xfds({0}),tv({0,0}),
    m_read_eagained_flag(false)
{
    m_serial_fd = -1;
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
        throw std::runtime_error(std::format("Failed to open serial device {}", dev.c_str()));
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
    m_read_buffer_uptr  = nullptr;
    m_write_buffer_uptr = nullptr;
    apply_default_settings(fd);
}

serial_bridge::SerialLink::~SerialLink()
{
    close(m_serial_fd);
}
#if 0
//void serial_bridge::SerialLink::guard_trigger_function()
//{
//    m_guard_condition_sptr->trigger();
//}
void serial_bridge::SerialLink::guard_condition_callback([[maybe_unused]]std::size_t n)
{
    IoBuffer::UPtr input_buffer_uptr  = std::make_unique<IoBuffer>();
    bool gotone = m_client_queue_uptr->get_nowait(input_buffer_uptr);
    if(gotone) {
        m_recv_callback(std::move(input_buffer_uptr));
    }
    return;
}
rclcpp::Logger serial_bridge::SerialLink::get_logger()
{
    return m_companion_node_ptr->get_logger();
}
#endif
void serial_bridge::SerialLink::send_threadsafe(IoBuffer::UPtr buffer_uptr) const
{
    // printf("send_threadsafe msg: %s\n", msg.c_str());
    m_output_queue_uptr->put(std::move(buffer_uptr));
}
void serial_bridge::SerialLink::run(OnRecvCallback on_recv_callback)
{
    int             retval;
    m_recv_callback = std::move(on_recv_callback);
    FD_SET(m_serial_fd, &m_rfds);
    FD_SET(m_output_queue_fd, &m_rfds);
    try {
        while (true) {
            FD_SET(m_serial_fd, &m_rfds);
            FD_SET(m_output_queue_fd, &m_rfds);
            m_nbr_fds = std::max(m_serial_fd, m_output_queue_fd) + 1;
            retval = select(m_nbr_fds, &m_rfds, &m_wfds, &m_xfds, nullptr);

            if (retval < 0) {
                int saved_errno = errno;
                // printf("select returned -1 errno is %d, %s", errno, strerror(errno));
                throw std::runtime_error(
                        std::format("select error retval: %d  errno: %d  strerror: %s", retval, saved_errno,
                                    strerror(saved_errno)));
            } else if (retval == 0) {
                throw std::runtime_error("select returned zero but no timeout was set");
            } else {
                printf("serial_link::run about to try write\n");
//                    auto x = FD_ISSET(m_output_queue_fd, &m_rfds);
                if(FD_ISSET(m_output_queue_fd, &m_rfds)) {
                    this->try_write();
                }
                if(m_write_buffer_uptr) {
                    this->try_write();
                }
                if (FD_ISSET(m_serial_fd, &m_rfds)) {
                    printf("try_read - select says there is data\n");
                    this->try_read();
                }
                if (!m_read_eagained_flag) {
                    printf("try_read - not eagain \n");
                    this->try_read();
                }
            }
        }
    } catch (std:: exception& e) {
        RBL_LOG_FMT("serial_link caught an exception  what: %s\n", e.what());
                       close(m_serial_fd);
    }
}
/**
 * The next 2 functions need a serious cleanup - maybe present the logic as nested FSM
*/
void serial_bridge::SerialLink::try_write() 
{
    FD_CLR(m_serial_fd, &m_wfds);
    if(m_write_buffer_uptr == nullptr) {
        printf("try_write getting from output queue\n");
        [[maybe_unused]]bool gotone = m_output_queue_uptr->get_nowait(m_write_buffer_uptr);
        if(gotone) {
            printf("try_write got this from output queue %s\n", m_write_buffer_uptr->to_string().c_str());
        } else {
            return;
        }
    }
    if(m_write_buffer_uptr) {
        printf("try_write output buffer %s\n", m_write_buffer_uptr->to_string().c_str());
        ssize_t n = write(m_serial_fd, m_write_buffer_uptr->data(), m_write_buffer_uptr->size());
        printf("try_write after write n: %ld\n", n);
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
void serial_bridge::SerialLink::try_read() 
{
    printf("try_read called \n");
    if((m_read_buffer_uptr != nullptr) && (!m_read_buffer_uptr->empty())) {
        printf("m_read_buffer_uptr is not null and not empty\n");
        assert(0);
        return;
    }
    m_read_buffer_uptr = std::make_unique<IoBuffer>();
    if(m_read_buffer_uptr->empty()) {
        // only do more reading when the read buffer has been processed to empty
        // auto b = m_read_buffer.space_ptr();
        if(m_read_buffer_uptr->space_len() == 0) {
            printf("m_read_buffer space_len == 0");
        }
        ssize_t n = read(m_serial_fd, m_read_buffer_uptr->space_ptr(), m_read_buffer_uptr->space_len());
        int saved_read_errno = errno;
        if(n == 0) {
            throw std::runtime_error(std::format("read returned zero errno: %d msg: %s\n", saved_read_errno, strerror(saved_read_errno)));
        } else if(n < 0 && saved_read_errno == EAGAIN) {
            m_read_eagained_flag = true;
        } else if(n < 0) {
            throw std::runtime_error(std::format("read returned -ve and no EAGAIN  errno: %d msg: %s\n", saved_read_errno, strerror(saved_read_errno)));
        } else {// n > 0 we read some data so commit it
            if(m_read_buffer_uptr->size() + n > m_read_buffer_uptr->capacity()) {
                throw std::runtime_error("no room in buffer to commit");
            }
            m_read_buffer_uptr->commit(n);
            printf("try_read got %s\n", m_read_buffer_uptr->to_string().c_str());
        }
    }
#if 1
    if(m_read_buffer_uptr && (! m_read_buffer_uptr->empty())) {
        m_parser.consume(*m_read_buffer_uptr, [this](IoBuffer::UPtr up){this->m_recv_callback(std::move(up));});
    }
#else
    // process or parse data
    if(m_read_buffer_uptr && (! m_read_buffer_uptr->empty())) {
        if(!m_input_message_buffer_uptr)
            m_input_message_buffer_uptr = std::make_unique<IoBuffer>();
        char* p;
        while((! m_read_buffer_uptr->empty()) && (p = m_read_buffer_uptr->get_first_char_ptr()) != nullptr) {
            char ch = *p;
            m_read_buffer_uptr->consume(1);
            if(ch == CARRIAGE_RETURN) {
                printf("got a CR character - these should not be present in the LINE protocol.");
            } else if(ch == LINE_FEED) {
                if(m_input_message_buffer_uptr->empty()) {
                    // RCLCPP_WARN(this->get_logger(), "We got a packet of no bytes ");
                } else {
                    m_recv_callback(std::move(m_input_message_buffer_uptr));
                    m_input_message_buffer_uptr = nullptr;
//                    std::size_t qn = m_client_queue_uptr->put(std::move(m_input_message_buffer_uptr));
                    m_input_message_buffer_uptr = std::make_unique<IoBuffer>();
                }
            } else {
                m_input_message_buffer_uptr->append(&ch, 1);
            }
        }

#endif
}
