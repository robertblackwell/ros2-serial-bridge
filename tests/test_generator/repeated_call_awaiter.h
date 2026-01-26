
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <thread>
#include <type_traits>
#include <functional>
#include <memory>
#include <coroutine>
#include <rbl/unittest.h>
#include <rbl/iobuffer.h>
#include <serial_link/serial_asio.h>
#include <serial_link/sync_serial_link.h>
#include <serial_link/serial_settings.h>
#include <boost/asio.hpp>
#include <boost/asio/experimental/coro.hpp>

using namespace rbl;
using namespace boost::asio;
using namespace boost::system;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

inline void serial_recv(serial_bridge::SerialAsio& asio_serial, std::string& dummy_args2, std::function<void(rbl::IoBuffer::UPtr up, error_code ec)> cb)
{
    // auto msg = std::string("This is a new iobuffer ");
    // cb(std::move(msg), true);
    asio_serial.recv([cb](rbl::IoBuffer::UPtr up, boost::system::error_code& ec)
    {
        cb(std::move(up), ec);
    });
}
class RepeatedCallAwaitable {
private:
    std::coroutine_handle<> m_handle;
    std::optional<IoBuffer::UPtr> m_msg;
    bool m_success = false;
    serial_bridge::SerialAsio& m_asio_serial;
    std::string& m_arg2;
public:
    RepeatedCallAwaitable(serial_bridge::SerialAsio& asio_serial, std::string& arg2):m_asio_serial(asio_serial), m_arg2(arg2)
    {};
    bool await_ready() const noexcept {return false;}
    void await_suspend(std::coroutine_handle<> handle)
    {
        m_handle = handle;
        // // serial_recv(m_asio_serial, m_arg2, [this](IoBuffer::UPtr up, error_code ec)
        // m_asio_serial.recv([this](IoBuffer::UPtr up, error_code ec)
        // {
        //
        //     m_msg = std::move(up);
        //     m_success = !ec;
        //     if (m_success) {
        //         m_handle.resume();
        //     }
        // });
    }
    IoBuffer::UPtr await_resume()
    {
        if (!m_success) {
            throw std::runtime_error{"failed to await"};
        }
        return std::move(*m_msg);
    }
    void resume_with_value(IoBuffer::UPtr up)
    {
        m_msg = std::move(up);
        m_success = true;
        m_handle.resume();
    }
};
struct Task {
    struct promise_type {
        RepeatedCallAwaitable yield_value(IoBuffer::UPtr up)
        {
            return RepeatedCallAwaitable{}
        }
        Task get_return_object() {return {};}
        std::suspend_never initial_suspend() {return {};}
        std::suspend_never final_suspend() noexcept {return {};}
        void return_void() {}
        void unhandled_exception() {}
    };
};

inline auto recv_as_coro(serial_bridge::SerialAsio& asio_serial, std::string& arg2) -> AsioRecvAwaitable
{
    auto coro = AsioRecvAwaitable{asio_serial, arg2};
    return coro;
}
inline Task perform_async_read(serial_bridge::SerialAsio& asio_serial)
{
    std::cout << "Starting sync read operation .." << std::endl;
    try {
        std::string arg1{"ThisIsArg1"};
        std::string arg2{"ThisIsArg2"};
        auto coro = AsioRecvAwaitable{asio_serial, arg2};
        for (int i = 0; i < 3; i++) {
            // std::string data = co_await AsyncReadAwaitable(arg1);
            IoBuffer::UPtr data = co_await coro;
            // std::string data = co_await recv_as_coro(arg1);
            std::cout << data->c_str() << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}
inline Task reader(serial_bridge::SerialAsio& asio_serial)
{
    std::cout << "Starting sync read operation .." << std::endl;
    try {
        std::string arg1{"ThisIsArg1"};
        std::string arg2{"ThisIsArg2"};
        auto recv_coro = AsioRecvAwaitable{asio_serial, arg2};
        for (;;) {
            // std::string data = co_await AsyncReadAwaitable(arg1);
            IoBuffer::UPtr data = co_await recv_coro;
            // std::string data = co_await recv_as_coro(arg1);
            std::cout << "reader got [" << data->c_str() << "]"<< std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}
#pragma GCC diagnostic pop