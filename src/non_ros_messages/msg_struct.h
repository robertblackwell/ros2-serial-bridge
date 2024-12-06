#include <string>
#include <vector>
#ifndef F710_CMD_RESPONSE_H
#define F710_CMD_RESPONSE_H
namespace non_ros_msgs {
    struct MotorPwmCmd {
        float left_motor_pwm;
        float right_motor_pwm;
    };
    struct MotorRpmCmd {
        float left_motor_rpm;
        float right_motor_rpm;
    };
    struct CmdResponse {
        bool ok;
        std::string text;
    };
    struct EncoderStatus {
        int64_t sample_sum;
        int64_t sample_timestamp_usecs;
        float   motor_rpm_estimate;
        bool    direction_pin_state;
    };
    struct LoadTestCmd {
        int32_t count;
        int32_t msg_length;
        int32_t msgs_per_second;
    };
    struct TextMsg {
        std::string text;
    };
    struct EchoCmd {
        std::vector<std::string> text;
    };
    struct ReadEncodersCmd {
        int left_or_right;
    };
    struct TwoEncoderStatus {
        EncoderStatus left;
        EncoderStatus right;
    };
}
#endif //F710_CMD_RESPONSE_H
