
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <thread>
#include <type_traits>

#include <unittest.h>
#include <iobuffer.h>
#include <serial_link/serial_link.h>
#include <serial_link/serial_settings.h>

using namespace rbl;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

struct Context {
    serial_bridge::SerialLink* serial_link_01;
    serial_bridge::SerialLink* serial_link_02;
};

void* thread_01(const std::string& dev)
{
    serial_bridge::SerialLink serial{dev, 1};
    std::thread t([&serial]()
    {

        serial.run([](IoBuffer::UPtr up)
        {
            std::cout << "YYYYYYYY  thread_01  got one bbbbbbb [" << up->c_str() << "]" << std::endl;
            up = nullptr;
        });
    });
    sleep(1);
    for (int i = 100; i < 130; i++) {
        // usleep(50000);
        IoBuffer::UPtr up = std::make_unique<IoBuffer>();
        auto n = snprintf((char*)up->space_ptr(), up->space_len(), "AAAA This is a message\n");
        up->commit(n);
        auto ss = std::format("{} AAAA This is a message\n", i);
        // std::cout << "thread_01 about to send " << ss << std::endl;
        auto up2 = std::make_unique<IoBuffer>(ss);
        serial.send_threadsafe(std::move(up2));
    }
    t.join();
    return nullptr;
}

void* thread_02(const std::string& dev)
{
    serial_bridge::SerialLink serial{dev, 2};
    std::thread t([&serial]()
    {
        serial.run([&serial](IoBuffer::UPtr up)
        {
            std::cout << "XXXXX thread 02 got one aaaaa [" << up->c_str() << "]" << std::endl;
            if (false) {
                // up = nullptr;
                auto s = std::format("[thread 02 got one aaaaa {}]", up->c_str());
                // std::cout << "thread 02 about to send " << s << std::endl;
                auto up2 = std::make_unique<IoBuffer>(s);
                up2->append("\n");
                // if (serial.m_instance_id == 2)
                //     printf("got here");
                serial.send_threadsafe(std::move(up2));
            }
        });
    });
    sleep(1);
    for (int i = 100; i < 130; i++) {
        // usleep(50000);
        auto nn = std::format("{} BBBBBBB  This is a message\n", i);
        IoBuffer::UPtr up = std::make_unique<IoBuffer>(nn);
        serial.send_threadsafe(std::move(up));
    }
    t.join();
    return nullptr;
}

int test_loop()
{
    auto ans = list_serial_devices("/dev", "ttyUSB");
    std::string dev1 = std::string{"/dev/ttyUSB0"};
    std::string dev2 = std::string{"/dev/ttyUSB1"};
    std::thread t1([dev1]() {
        thread_01(dev1);
    });
    std::thread t2([&dev2]()
    {
        thread_02(dev2);
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