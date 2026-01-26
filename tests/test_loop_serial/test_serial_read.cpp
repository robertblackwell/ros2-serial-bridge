
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <thread>
#include <type_traits>

#include <rbl/unittest.h>
#include <rbl/iobuffer.h>
#include <serial_link/serial_link.h>
#include <serial_link/sync_serial_link.h>
#include <serial_link/serial_settings.h>
#include "run_data.h"

using namespace rbl;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

struct Context {
    serial_bridge::SerialLink* serial_link_01;
    serial_bridge::SerialLink* serial_link_02;
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

void* reader_thread_01(const std::string& dev, RunData* run_data)
{
    serial_bridge::SerialLink serial{dev, 1};
    serial.run([run_data](IoBuffer::UPtr up)
    {
        std::string expt = expected(make_test_case(run_data->index));
        run_data->update(*up);
        bool ok = (std::string{up->c_str()} == expt);

        assert(ok);
        up = nullptr;
        if (run_data->index == run_data->max_index) {
            run_data->end();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>((run_data->end_time) - run_data->start_time);
            auto d = (float)duration.count();
            auto cps = ((float)(run_data->char_count) * 1000.0) / d;
            std::cout << "chars: " <<  run_data->char_count << " millisecs: "<< duration.count() << " chars / sec: " << cps << std::endl;
            exit(0);
        }
    });
    return nullptr;
}

void* writer_thread_02(const std::string& dev, RunData* run_data)
{
    serial_bridge::SyncSerialLink sync_serial{dev};
    usleep(250000);
    for (int i = 100; i <= run_data->max_index; i++) {
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
    auto run_data = new RunData();
    std::thread t1([dev1, run_data]() {
        reader_thread_01(dev1, run_data);
    });
    std::thread t2([&dev2, run_data]()
    {
        writer_thread_02(dev2, run_data);
    });
    t1.join();
    t2.join();
    delete run_data;
    return 0;
}
int main()
{
    UT_ADD(test_loop);
    int rc = UT_RUN();
    return rc;
}
#pragma GCC diagnostic pop