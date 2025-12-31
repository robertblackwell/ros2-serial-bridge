#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <optional>
#include <format>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/jsonpath/jsonpath.hpp>

#include <msgs/msgs.h>
#include <rbl/iobuffer.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

void test_01()
{
    printf("Hello this is test_msgs\n");
	rbl::IoBuffer buffer{};
	buffer.append(std::string{R"({"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"})"});
	printf("json string is %s\n", buffer.to_string().c_str());
    std::string json_string = buffer.to_string();
    jsoncons::json j = jsoncons::json::parse(json_string);
    EncoderStatus status{};
    status.sample_sum = j["ss"].as<int64_t>();
    status.sample_time_stamp_usecs = j["ts"].as<int64_t>();
    status.motor_rpm_estimate = j["mr"].as<float>();

    std::string x = j["ps"].as<std::string>();

}
void test_02()
{
    printf("Hello this is test_msgs\n");
	rbl::IoBuffer buffer{};
	buffer.append(std::string{R"([{"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"}, {"ss":77777777, "ts":129283746, "mr":5767.43, "ps":120}])"});
	printf("json string is %s\n", buffer.to_string().c_str());
    std::string json_string = buffer.to_string();
    jsoncons::json j = jsoncons::json::parse(json_string);
    TwoEncoderStatus status{};
    status.left_sample_sum              = j[0]["ss"].as<int64_t>();
    status.left_sample_time_stamp_usecs = j[0]["ts"].as<int64_t>();
    status.left_motor_rpm_estimate      = j[0]["mr"].as<float>();
    status.right_sample_sum              = j[1]["ss"].as<int64_t>();
    status.right_sample_time_stamp_usecs = j[1]["ts"].as<int64_t>();
    status.right_motor_rpm_estimate      = j[1]["mr"].as<float>();

    std::string x0 = j[0]["ps"].as<std::string>();
    std::string x1 = j[1]["ps"].as<std::string>();

}

void f(CmdResponse& resp)
{
    printf("Do something with a cmd response\n");
}
void f(EncoderStatus& status)
{
    printf("Do something with a encoder response\n");
}
void f(TwoEncoderStatus& status)
{
    printf("Do something with a 222 Two encoder response\n");
}
void f(TextMsg& resp)
{
    printf("Do something with a text msg\n");
}
struct overloadf
{
    void operator()(CmdResponse& inmsg){f(inmsg);}
    void operator()(EncoderStatus& inmsg){f(inmsg);}
    void operator()(TwoEncoderStatus& inmsg){f(inmsg);}
    void operator()(TextMsg& inmsg){f(inmsg);}
};
void test_03()
{
	rbl::IoBuffer buffer{};
	buffer.append(std::string{R"({"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"})"});
    EncoderStatus status{};
    deserialize(buffer, status);
    InputMessage vv{status};
    std::visit(overloadf{}, vv);
    printf("we are done");
}
void test_04()
{
	rbl::IoBuffer buffer{};
	buffer.append(std::string{R"(1K[{"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"}, {"ss":77777777, "ts":129283746, "mr":5767.43, "ps":120}])"});
    EncoderStatus status{};
    // deserialize(buffer, status);
    // InputMessage vv{status};
    InputMessage vv{};
    deserialize(buffer, vv);
    if(std::holds_alternative<TwoEncoderStatus>(vv)) {
        printf("222TwoEncoderStatus \n");
    } else if(std::holds_alternative<EncoderStatus>(vv)) {
        printf("EncoderStatus \n");
    } else if(std::holds_alternative<TextMsg>(vv)) {
        printf("Text Msg \n");
    }
    if(auto x = std::get_if<TwoEncoderStatus>(&vv)) {
        printf("222TwoEncoderStatus \n");
    } else if(auto x = std::get_if<EncoderStatus>(&vv)) {
        printf("EncoderStatus \n");
    } else if(auto x = std::get_if<TextMsg>(&vv)) {
        printf("Text Msg \n");
    }
    printf("we are done");
}
void test_05()
{
    auto make_iobuffer = [](OutputMessage msg) {
        rbl::IoBuffer::UPtr buf_uptr = std::make_unique<rbl::IoBuffer>();
        serialize(msg, *buf_uptr);
        return buf_uptr;
    };

    auto echo = EchoCmd{};
    echo.data = std::vector<std::string>({std::string("1111"),std::string("2222")});
    for(auto &s : echo.data) {
        const char* tmp = s.c_str();
        printf("%s\n", tmp);
    }
    EchoCmd echo_cmd{}; echo_cmd.data = std::vector<std::string>({std::string("AAAAAA"),std::string("BBBBBB")});

    auto echo2 = make_iobuffer(echo_cmd);

    printf("We r done %s\n", echo2->c_str());
}
int main(int argc, char * argv[])
{
    printf("Hello this is test_msgs\n");
    test_05();
    test_04();
    test_03();
    test_01();
    test_02();

	return 0;
}
#pragma GCC diagnostic push
