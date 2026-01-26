#ifndef H_serial_test_run_data_H
#define H_serial_test_run_data_H
#include <chrono>
#include <boost/system/error_code.hpp>
#include <rbl/iobuffer.h>
#include <cassert>

struct RunData {
    int index;
    int max_index;
    int char_count;
    double cps;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    RunData()
    {
        index = 100;
        max_index = 1500;
        char_count = 0;
        start_time = std::chrono::high_resolution_clock::now();
        end_time = std::chrono::high_resolution_clock::now();
        cps = 0.0;
    }
    void start()
    {
        start_time = std::chrono::high_resolution_clock::now();
        end_time = std::chrono::high_resolution_clock::now();
    }
    void end()
    {
        end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>((end_time) - start_time);
        auto d = (float)duration.count();
        cps = ((float)(char_count) * 1000.0) / d;
    }
    void update(rbl::IoBuffer::UPtr up, const boost::system::error_code& ec)
    {
        if (!ec) {
            index++;
            std::cout << "do_read_loop.start cb [" << up->c_str() << "]" << "\n";
        } else {
            std::cerr << ec.message() << "\n";
            assert(0);
        }
    }
    void update(const rbl::IoBuffer& iob)
    {
        index++;
        char_count += static_cast<int>(iob.size());
        // std::cout << "do_read_loop.start cb [" << iob.c_str() << "]" << "\n";
    }
};


#endif