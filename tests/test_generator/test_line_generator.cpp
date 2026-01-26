
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <thread>
#include <generator>
#include <rbl/unittest.h>
#include <rbl/iobuffer.h>

using namespace rbl;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <boost/asio/read_until.hpp>
#include <sstream>
#include <exception>
#include <serial_link/sync_serial_link.h>

using namespace boost::asio;
using namespace boost::asio::placeholders;

// A generator that yields lines from the serial port.
awaitable<std::string> async_read_lines(serial_port& port, boost::asio::streambuf& buffer) {
    try {
        while (true) {
            // Asynchronously read data until a newline character is encountered.
            // When this operation suspends, control returns to the caller's event loop.
            // When it resumes, the result is available and assigned to 'n'.
            std::size_t n = co_await async_read_until(port, buffer, '\n', use_awaitable);

            // Use an input stream to extract the line from the streambuf.
            std::istream is(&buffer);
            std::string line;
            std::getline(is, line);

            // Process the line to remove the newline character (or carriage return if present)
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            // Yield the completed line to the consumer of this coroutine.
            // Note: std::string cannot be co_yielded directly with standard C++20 generator utility types.
            // This example uses a conceptual 'awaitable<std::string>' which in a real implementation
            // needs a proper coroutine type (like cppcoro::generator or boost::asio::awaitable)
            // to support 'co_yield' a value for each iteration. For simplicity, we adapt
            // the async_read_until logic in an awaitable task that processes one line at a time.
            // To make this function yield iteratively, it would need a custom generator promise type.
            // The provided code below is a more direct way to use co_await in a loop.

            // Re-throwing the line as an awaitable result for the demonstration's sake (not a real generator)
            // The actual logic would involve a custom generator type.

            // A more practical approach is to put the consumer logic directly within this co_spawn.
            // We can't actually 'co_yield' here in a standard way that a simple for loop could consume
            // without a custom generator implementation.

            // Instead, this code block demonstrates the async reading part.
            // A working generator would look different.
        }
    } catch (const boost::system::system_error& e) {
        std::cerr << "Serial port error: " << e.what() << std::endl;
        // In a real application, handle port closure/error gracefully.
    }
}

// A more practical example that processes the lines within the spawned coroutine.
awaitable<void> process_serial_data(serial_port& port) {
    boost::asio::streambuf buffer;
    try {
        while (true) {
            std::size_t n = co_await async_read_until(port, buffer, '\n', use_awaitable);
            std::istream is(&buffer);
            std::string line;
            std::getline(is, line);
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            std::cout << "Received line: " << line << std::endl;
        }
    } catch (const boost::system::system_error& e) {
        if (e.code() != boost::asio::error::operation_aborted) {
            std::cerr << "Serial port error in processing: " << e.what() << std::endl;
        }
    }
}
using MyVariant = std::variant<std::string, int>;
std::vector<MyVariant> make_test_case(int id)
{
    std::vector<MyVariant> tc = {std::format("{}: ABCDEFGHIJKLMNOP1234567890\n", id)};
    return tc;
}
template<class... Ts>
struct overloaded: Ts... {
    using Ts::operator()...;
};

void* writer_thread_02()
{
    const std::string& dev = "/dev/ttyUSB0";
    int max_count = 1000;
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
    sleep(3);
    exit(0);
    return nullptr;
}


int reader() {
    try {
        boost::asio::io_context io_context;

        // Open the serial port. Change "COM3" or "/dev/ttyS0" to your port name.
        boost::asio::serial_port port(io_context, "/dev/ttyUSB1");

        // Configure the serial port (e.g., 9600 baud rate)
        port.set_option(boost::asio::serial_port_base::baud_rate(115200));
        port.set_option(boost::asio::serial_port_base::character_size(8));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        std::cout << "Serial port opened. Waiting for data..." << std::endl;

        // Spawn the coroutine to process data.
        co_spawn(io_context, process_serial_data(port), detached);

        // Run the I/O context. This call blocks until all work is finished.
        io_context.run();
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

int main()
{
    std::thread t1(reader);
    writer_thread_02();
    t1.join();
    return 0;
}

#pragma GCC diagnostic pop