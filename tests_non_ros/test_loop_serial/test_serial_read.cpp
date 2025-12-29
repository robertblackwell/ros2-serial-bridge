
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <thread>
#include <type_traits>

#include <unittest.h>
#include <iobuffer.h>
#include <serial_link/serial_link.h>
#include <serial_link/sync_serial_link.h>
#include <serial_link/serial_settings.h>

using namespace rbl;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

struct Context {
    serial_bridge::SerialLink* serial_link_01;
    serial_bridge::SerialLink* serial_link_02;
};

using MyVariant = std::variant<std::string, int>;
std::vector<MyVariant> s1 = {"ABC", 10, "CD/n"};

template<class... Ts>
struct overloaded: Ts... {
    using Ts::operator()...;
};

void* reader_thread_01(const std::string& dev)
{
    serial_bridge::SerialLink serial{dev};
    serial.run([](IoBuffer::UPtr up)
    {
        std::cout << "reader_thread_01  got one bbbbbbb " << up->c_str() << std::endl;
        up = nullptr;
    });
    return nullptr;
}

void* writer_thread_02(const std::string& dev)
{
    serial_bridge::SyncSerialLink sync_serial{dev};
    usleep(250000);
    for (int i = 100; i < 10000; i++) {
        usleep(150000);
        for (auto element: s1) {
            std::visit(overloaded{
                [](int delay) {sleep(delay);},
                [&sync_serial](std::string s){sync_serial.send(const_cast<char*>(s.c_str()), s.length());}
            }, element);
        }
        // IoBuffer::UPtr up(new IoBuffer());
        // auto nn = snprintf((char*)up->space_ptr(), up->space_len(), "%d BBBBBBB  This is a message\n", i);
        // up->commit(nn);
        // sync_serial.send(static_cast<char*>(up->data()), up->data_len());
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
    UT_ADD(test_loop);
    int rc = UT_RUN();
    return rc;
}
#pragma GCC diagnostic pop