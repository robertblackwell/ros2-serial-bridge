
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
#include "asio_serial_coro.h"

using namespace rbl;
using namespace boost::asio;
using namespace boost::system;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

struct Context {
    serial_bridge::SerialAsio* serial_asio_01;
    serial_bridge::SerialAsio* serial_asio_02;
};

using MyVariant = std::variant<std::string, int>;
std::vector<MyVariant> test_case1 = {"ABCDEFGHIJKLMNOP", 2, "1234567890\n"};

std::vector<MyVariant> make_test_case_with_delay(int id)
{
    std::vector<MyVariant> tc = {std::format("{}: ABCDEFGHIJKLMNOP", id), 250000, "1234567890\n"};
    return tc;
}
std::vector<MyVariant> make_test_case(int id)
{
    std::vector<MyVariant> tc = {std::format("{}: ABCDEFGHIJKLMNOP1234567890\n", id)};
    return tc;
}
template<class... Ts>
struct overloaded: Ts... {
    using Ts::operator()...;
};

std::string expected(std::vector<MyVariant> input)
{
    std::string result{};
    for (auto bit : input) {
        std::visit(overloaded{
            [](int delay) {;},
            [&result](std::string s){result += s;}
        }, bit);
    }
    result.pop_back();
    return result;
}
void* reader_thread_01(const std::string& dev)
{
    io_context ioctx;
    serial_bridge::SerialAsio serial{dev, 1, ioctx};
    int index = 100;
    serial.recv([](rbl::IoBuffer::UPtr up, boost::system::error_code& ec)
    {
        std::cout << up->c_str() << std::endl;
    });
    ioctx.run();
    return nullptr;
}

void* writer_thread_02(const std::string& dev)
{
    serial_bridge::SyncSerialLink sync_serial{dev};
    usleep(250000);
    for (int i = 100; i <= 1000; i++) {
        // usleep(15000);
        auto tc = make_test_case(i);
        for (auto element: tc) {
            std::visit(overloaded{
                [](int delay) {usleep(delay);},
                [&sync_serial](std::string s){sync_serial.send(const_cast<char*>(s.c_str()), s.length());}
            }, element);
        }
    }
    sleep(30);
    return nullptr;
}

int test_loop()
{
    auto ans = list_serial_devices("/dev", "ttyUSB");
    std::string dev1 = std::string{"/dev/ttyUSB0"};
    std::string dev2 = std::string{"/dev/ttyUSB1"};

    std::thread t1([dev1]() {
        reader_thread_01(dev1);
    });
    std::thread t2([&dev2]()
    {
        writer_thread_02(dev2);
    });
    t1.join();
    t2.join();
    return 0;
}
int main()
{
    auto dev1 = std::string{"/dev/ttyUSB0"};
    auto dev2 = std::string{"/dev/ttyUSB1"};
    std::thread t1([dev1]()
    {
        sleep(2);
        writer_thread_02(dev1);
    });
    std::thread t2([&dev2]()
    {
        io_context ioctx;
        serial_bridge::SerialAsio serial{dev2, 1, ioctx};
        if (0) {
            post(ioctx, [&serial]()
            {
                serial.recv([&](rbl::IoBuffer::UPtr up, error_code& ec)
                {
                    std::cout << up->c_str() << std::endl;
                });
            });
        } else {
            reader(serial);
        }
        ioctx.run();
    });
    t1.join();
    t2.join();
    return 0;
}
#pragma GCC diagnostic pop