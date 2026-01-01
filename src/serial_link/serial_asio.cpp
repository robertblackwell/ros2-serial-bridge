#include <utility>
#include <cassert>
#include <cerrno>
#include <format>
#include <functional>
#include <memory>
#include <sys/select.h>
#include <chrono>
#include <rbl/logger.h>
#include "serial_asio.h"
#include "serial_settings.h"

#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'

serial_bridge::SerialAsio::SerialAsio(const std::string& dev, int instance_id, asio::io_context& io_context )
    : m_io_context(io_context), m_serial_fd(open_serial_non_blocking(dev)),
    m_asio_serial_port(asio::serial_port(io_context, m_serial_fd))
{
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
    apply_default_settings(m_serial_fd);
    m_read_is_active = false;
    m_read_cb = nullptr;
    m_read_buffer_uptr = std::make_unique<IoBuffer>();
    m_write_buffer_uptr = nullptr;
}

serial_bridge::SerialAsio::~SerialAsio()
{
    close(m_serial_fd);
}
void serial_bridge::SerialAsio::send(IoBuffer::UPtr msg_buffer_uptr)
{
    assert(msg_buffer_uptr != nullptr);
    // transform the msg buffer to a protocol buffer - add the trailing \n
    asio::post(m_io_context, [this, captured_buffer_uptr = std::move(msg_buffer_uptr)]() mutable
    {
        assert(captured_buffer_uptr != nullptr);
        m_write_buffer_queue.push(std::move(captured_buffer_uptr));
        if (m_write_buffer_uptr == nullptr) {
            try_write();
        }
    });
}
void serial_bridge::SerialAsio::recv(std::function<void(rbl::IoBuffer::UPtr up, boost::system::error_code& ec)> cb)
{
    if (m_read_cb != nullptr) {
        auto erc = boost::system::errc::make_error_code(boost::system::errc::operation_in_progress);
        assert(cb != nullptr);
        cb(nullptr, erc);
        return;
    }
    // std::cout << "q size: " << m_read_msg_queue.size() << std::endl;
    if (!m_read_msg_queue.empty()) {
        auto msg = std::move(m_read_msg_queue.front());
        m_read_msg_queue.pop();
        boost::system::error_code ec;
        assert(cb != nullptr);
        cb(std::move(msg), ec);
        return;
    }
    m_read_is_active = true;
    m_read_buffer_uptr = std::make_unique<IoBuffer>(1024);
    m_read_cb = cb;
    auto b = asio::buffer(m_read_buffer_uptr->space_ptr(), m_read_buffer_uptr->space_len());
    m_asio_serial_port.async_read_some(b, [this](const boost::system::error_code& ec, std::size_t bytes_transferred)
    {
        if (ec) {
            throw std::runtime_error(ec.message());
        }
        assert(bytes_transferred != 0);
        m_read_buffer_uptr->commit(bytes_transferred);
        m_parser.consume(*m_read_buffer_uptr, [this](IoBuffer::UPtr msg_buf_up)
        {
            assert(msg_buf_up != nullptr);
            m_read_msg_queue.push(std::move(msg_buf_up));
        });
        assert(m_read_buffer_uptr->empty());
        // std::cout << "parser done q size: " << m_read_msg_queue.size() << std::endl;
        boost::system::error_code localec;
        assert(m_read_cb != nullptr);
        auto tmp_msg = std::move(m_read_msg_queue.front());
        m_read_msg_queue.pop();
        auto tmp_cb = m_read_cb;
        m_read_cb = nullptr;
        m_read_is_active = false;
        tmp_cb(std::move(tmp_msg), localec);
    });
}
void serial_bridge::SerialAsio::try_write()
{
    assert(m_write_buffer_uptr == nullptr);
    if (m_write_buffer_queue.empty()) {
        return;
    }
    m_write_buffer_uptr = std::move(m_write_buffer_queue.front());
    m_write_buffer_queue.pop();
    auto bb = asio::buffer((char*)m_write_buffer_uptr->data(), m_write_buffer_uptr->data_len());
    asio::async_write(m_asio_serial_port, bb, [this](boost::system::error_code ec, std::size_t n)
    {
        assert(n == m_write_buffer_uptr->data_len());
        if (!ec) {
            m_write_buffer_uptr->consume(n);
            assert(m_write_buffer_uptr->empty());
            return;
        } else {
            throw std::runtime_error(ec.message());
        }
    });
}


void serial_bridge::SerialAsio::run_start(OnRecvCallback cb)
{
    recv([this, &cb](IoBuffer::UPtr up, boost::system::error_code& ec)
    {
        if (!ec) {
            cb(std::move(up), ec);
            asio::post(m_io_context, [this, &cb]()
            {
                run_start(cb);
            });
            return;
        }
        assert(0);
    });

}
void serial_bridge::SerialAsio::runner(std::function<void(IoBuffer::UPtr up, boost::system::error_code& ec)> cb)
{
    recv([this, cb](IoBuffer::UPtr up, boost::system::error_code& ec)
    {
        if (!ec) {
            cb(std::move(up), ec);
            asio::post(m_io_context, [this, &cb]()
            {
                runner(cb);
            });
        } else {
            std::cerr << ec.message() << "\n";
            assert(0);
        }
    });
}

void serial_bridge::SerialAsio::run(const OnRecvCallback& cb)
{
    runner(cb);
}
