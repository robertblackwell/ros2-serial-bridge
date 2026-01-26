
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

using namespace rbl;
using namespace boost::asio;
using namespace boost::system;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

class BasicCoroutine {
    public:
    struct Promise {
        BasicCoroutine get_return_object() {return BasicCoroutine {};}
        void unhandled_exception() noexcept {}
        void return_void() noexcept {}
        std::suspend_never initial_suspend() noexcept {return {};}
        std::suspend_never final_suspend() noexcept {return {};}
    };
    using promise_type = Promise;
};
BasicCoroutine coro() {co_return;}
int main()
{
    coro();
    return 0;
}
/* schematic of a coroutine
ReturnType someCoroutine(Parameters parameter)
{
    auto* frame = new coroutineFrame(std::forward<Parameters>(parameters));
    auto returnObject = frame->promise.get_return_object();
    co_await frame->promise.initial_suspend();
    try
    {
        <body-statements>
    }
    catch (...)
    {
        frame->promise.unhandled_exception();
    }
    co_await frame->promise.final_suspend();
    delete frame;
    return returnObject;
}
*/
#pragma GCC diagnostic pop