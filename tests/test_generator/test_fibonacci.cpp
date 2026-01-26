
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <generator>
#include <rbl/unittest.h>
#include <rbl/iobuffer.h>

using namespace rbl;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

std::generator<int> fibonacci()
{
    co_yield 0;
    int a =0, b = 1;
    while (true) {
        co_yield b;
        int next = a + b;
        a = b;
        b = next;
    }
}


int main()
{
    std::cout << "First 10 fibonacci numbers" << std::endl;
    for (auto f: fibonacci() | std::views::take(10)) {
        std::cout << f << std::endl;
    }
    return 0;
}
#pragma GCC diagnostic pop