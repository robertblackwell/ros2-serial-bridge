
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <thread>
#include <unittest.h>
#include <iobuffer.h>
#include <serial_link/serial_link.h>

using namespace rbl;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

struct Context {
    serial_bridge::SerialLink* serial_link_01;
    serial_bridge::SerialLink* serial_link_02;
};

void* thread_01()
{
    serial_bridge::SerialLink serial{};
    std::thread t([&serial]()
    {
        serial.run([](IoBuffer::UPtr up)
        {
            std::cout << "bbbbbbb " << up->c_str() << std::endl;
            up = nullptr;
        });
    });
    sleep(2);
    for (int i = 100; i < 10000; i++) {
        sleep(2);
        IoBuffer::UPtr up(new IoBuffer());
        snprintf((char*)up->space_ptr(), up->space_len(), "AAAA This is a message");
        serial.send_threadsafe(std::move(up));
    }
    t.join();
    return nullptr;
}

void* thread_02()
{
    serial_bridge::SerialLink serial{};
    std::thread t([&serial]()
    {
        serial.run([](IoBuffer::UPtr up)
        {
            std::cout << "aaaaa " << up->c_str() << std::endl;
            up = nullptr;
        });
    });
    sleep(2);
    for (int i = 100; i < 10000; i++) {
        sleep(2);
        IoBuffer::UPtr up(new IoBuffer());
        snprintf((char*)up->space_ptr(), up->space_len(), "BBBBBBB  This is a message");
        serial.send_threadsafe(std::move(up));
    }
    t.join();
    return nullptr;
}

int test_loop()
{
    std::thread t1(thread_01);
    std::thread t2(thread_02);
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