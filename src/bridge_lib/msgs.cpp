#include <format>
#include "jsoncons/json.hpp"
#include "jsoncons_ext/jsonpath/jsonpath.hpp"
#include "msgs.h"
#include "iobuffer.h"

using namespace sample_interfaces::msg;
using IoBuffer = ros2_bridge::IoBuffer;

// Message deserializer(IoBuffer& message_buffer)
// {
//     if(message_buffer.empty()){
//         return Message("be warner deserialize got an empty IoBuffer");
//     }
//     return message_buffer.to_string();
// }

// void serializer(Message message, IoBuffer& iobuffer)
// {
//     if(! iobuffer.empty()) {
//         throw std::runtime_error("serialize iobuffer should be empty");
//     }
//     iobuffer.append(message);
// }
/**
 * Deserialize
*/

bool test_buffer_prefix(IoBuffer& buffer, std::string prefix)
{
    std::string test = buffer.substr(0, 2);
    // printf("%s  %s\n", test.c_str(), prefix.c_str());
    bool x = (test  == prefix);
    return x;
}
bool deserialize(IoBuffer& buffer, CmdResponse& cmd_response)
{
    if(! test_buffer_prefix(buffer, "1P")) {
        return false;
    }
    std::string s = buffer.to_string();
    cmd_response.ok = s.find("OK") != std::string::npos;
    cmd_response.text = s;
    return true;
}

bool deserialize(IoBuffer& buffer, EncoderStatus& status)
{
    if(! test_buffer_prefix(buffer, "1K")) {
        return false;
    }try {
        std::string json_string = buffer.to_string().substr(2, std::string::npos);
        jsoncons::json j = jsoncons::json::parse(json_string);
        status.sample_sum = j["ss"].as<int64_t>();
        status.sample_time_stamp_usecs = j["ts"].as<int64_t>();
        status.motor_rpm_estimate = j["mr"].as<float>();
        status.direction_pin_state = j["ps"].as<int8_t>();
        return true;
    } catch(std::exception& e) {
        return false;
    }
    // status.direction_pin_state = j["ps"].as<uint8_t>();
}
bool deserialize(IoBuffer& buffer, TwoEncoderStatus& status)
{
    if(! test_buffer_prefix(buffer, "1J")) {
        return false;
    }
    printf("deserialize TwoEncoderStatus %s\n", buffer.to_string().c_str());
    std::string json_string = buffer.to_string().substr(2,std::string::npos);
    jsoncons::json j = jsoncons::json::parse(json_string);
    status.left.sample_sum = j[0]["ss"].as<int64_t>();
    status.left.sample_time_stamp_usecs = j[0]["ts"].as<int64_t>();
    status.left.motor_rpm_estimate = j[0]["mr"].as<float>();
    status.left.direction_pin_state = j[0]["ps"].as<int8_t>();

    status.right.sample_sum = j[1]["ss"].as<int64_t>();
    status.right.sample_time_stamp_usecs = j[1]["ts"].as<int64_t>();
    status.right.motor_rpm_estimate = j[1]["mr"].as<float>();
    status.right.direction_pin_state = j[1]["ps"].as<int8_t>();
    return true;
}

bool deserialize(IoBuffer& buffer, TextMsg& text_msg)
{
    text_msg.text = buffer.to_string();
    return true;
}

bool deserialize(IoBuffer& buffer, InputMessage& inmsg)
{
    std::vector<std::function<bool(IoBuffer&)>> pv = {
        [&](IoBuffer& buffer){
            CmdResponse msg;
            if(deserialize(buffer, msg)) {
                inmsg = msg;
                return true;
            }
            return false;
        },
        [&](IoBuffer& buffer){
            EncoderStatus msg;
            if(deserialize(buffer, msg)) {
                inmsg = msg;
                return true;
            }
            return false;
        },
        [&](IoBuffer& buffer){
            TwoEncoderStatus msg;
            if(deserialize(buffer, msg)) {
                inmsg = msg;
                return true;
            }
            return false;
        },
        [&](IoBuffer& buffer){
            TextMsg msg;
            if(deserialize(buffer, msg)) {
                inmsg = msg;
                return true;
            }
            return false;
        },                
    };
    for(auto f : pv) {
        if(f(buffer))
            return true;
    } 
    return false;
}

/**
 * Serialize functions
*/
void serialize(const CmdResponse& response, IoBuffer& buffer)
{
    buffer.clear();
    auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "%s %s", (response.ok ? "OK": "FAILED"), response.text.c_str());
    buffer.commit(len);

}
void serialize(EchoCmd& cmd, IoBuffer& buffer)
{
    buffer.clear();
    std::string result{};
    for(auto s : cmd.data) {
        result +=  " " + s;
    }
    // auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "echo %s\n", result.c_str());
    buffer.append("echo "); buffer.append(result); 
    // buffer.commit(len);

}
void serialize(LoadTestCmd& cmd, IoBuffer& buffer)
{
    buffer.clear();
    auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "load %d %d %d", cmd.count, cmd.msg_length, cmd.msgs_per_seecond);
    buffer.commit(len);
}
void serialize(MotorPwmCmd& cmd, IoBuffer& buffer)
{
    buffer.clear();
    auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "pwm %f %f", cmd.left_motor_pwm, cmd.right_motor_pwm);
    buffer.commit(len);
}
void serialize(MotorRpmCmd& cmd, IoBuffer& buffer)
{
    buffer.clear();
    auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "rpm %f %f", cmd.left_motor, cmd.right_motor);
    buffer.commit(len);
}
void serialize(TextMsg& text_msg, IoBuffer& buffer)
{
    buffer.clear();
    // auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "%s", cmd_response.text);
    buffer.append(text_msg.text);
}
void serialize(ReadEncodersCmd& cmd, IoBuffer& buffer)
{
    buffer.clear();
    auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "e %d", cmd.n);
    buffer.commit(len);
}

void serialize(OutputMessage& msg, IoBuffer& buffer)
{
    if(auto m_ptr = std::get_if<EchoCmd>(&msg)) {
        serialize(*m_ptr, buffer);
    } else if(auto m_ptr = std::get_if<LoadTestCmd>(&msg)) {
        serialize(*m_ptr, buffer);
    } else if(auto m_ptr = std::get_if<MotorPwmCmd>(&msg)) {
        serialize(*m_ptr, buffer);
    } else if(auto m_ptr = std::get_if<MotorRpmCmd>(&msg)) {
        serialize(*m_ptr, buffer);
    } else if(auto m_ptr = std::get_if<TextMsg>(&msg)) {
        serialize(*m_ptr, buffer);
    } else if(auto m_ptr = std::get_if<ReadEncodersCmd>(&msg)) {
        serialize(*m_ptr, buffer);
    } else {
        throw std::runtime_error("serialize else branch");
    }
    buffer.append("\n");
}