#include <gtest/gtest.h>

TEST(cpp_serial_bridge, test_01)
{
    ASSERT_EQ(4, 2+2);
}
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}