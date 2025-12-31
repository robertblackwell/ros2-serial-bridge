
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
const int max_count = 1000;
auto start_time = std::chrono::high_resolution_clock::now();
std::chrono::time_point<std::chrono::high_resolution_clock> end_time_g;
void* reader_thread_01(const std::string& dev)
{
    serial_bridge::SerialLink serial{dev, 1};
    int index = 100;
    int max_index = max_count;
    int char_count = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock>* end_time_ref = &end_time_g;
    serial.run([&index, max_index, end_time_ref, &char_count](IoBuffer::UPtr up)
    {
        std::string expt = expected(make_test_case(index++));
        bool ok = (std::string{up->c_str()} == expt);
        char_count += static_cast<int>(up->data_len());
        assert(ok);
        // std::cout << "reader_thread_01  got one bool: " << ok << "   bbbbbbb " << up->c_str() << std::endl;
        up = nullptr;
        if (index == max_index) {
            *end_time_ref = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>((*end_time_ref) - start_time);
            std::cout << "chars: " <<  char_count << " millisecs: "<< duration.count() << " chars / sec: " << (char_count/duration.count())*1000 << std::endl;
            exit(0);
        }
    });
    return nullptr;
}

void* writer_thread_02(const std::string& dev)
{
    serial_bridge::SyncSerialLink sync_serial{dev};
    usleep(250000);
    for (int i = 100; i <= max_count; i++) {
        // usleep(15000);
        auto tc = make_test_case(i);
        for (auto element: tc) {
            std::visit(overloaded{
                [](int delay) {usleep(delay);},
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