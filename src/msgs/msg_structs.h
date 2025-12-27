//
// Created by robert on 12/27/25.
//

#ifndef CPP_SERIAL_BRIDGE_CMD_RESPONSE_H
#define CPP_SERIAL_BRIDGE_CMD_RESPONSE_H
#include <string>
struct CmdResponse {
    static constexpr std::string prefix = "1P";
    std::string ok;
    std::string text;
};
struct EncoderStatus {
    static constexpr std::string prefix = "1K";
    int64_t sample_sum;
    int64_t sample_time_stamp_usecs;
    float   motor_rpm_estimate;
    bool    direction_pin_state;
};
 struct TwoEncoderStatus {
     static constexpr std::string prefix = "1J";

     int64_t left_sample_sum;
     int64_t left_sample_time_stamp_usecs;
     float   left_motor_rpm_estimate;
     bool    left_direction_pin_state;

     int64_t right_sample_sum;
     int64_t right_sample_time_stamp_usecs;
     float   right_motor_rpm_estimate;
     bool    right_direction_pin_state;
};

struct EchoCmd {
    static constexpr std::string code = "e";
    std::vector<std::string> data;
};

struct LoadTestCmd {
    static constexpr std::string code = "load";
    int count;
    int msg_length;
    int msgs_per_seecond;
};

struct MotorPwmCmd {
    static constexpr std::string code = "pwm";
    float left_motor_pwm;
    float right_motor_pwm;
};

struct MotorRpmCmd {
    static constexpr std::string code = "rpm";
    float left_motor_rpm;
    float right_motor_rpm;
};

struct ReadEncodersCmd {
    static constexpr std::string code = "e";
    int n;
};

struct TextMsg {
    static constexpr std::string code = "t";
    std::string text;
};
#endif //CPP_SERIAL_BRIDGE_CMD_RESPONSE_H