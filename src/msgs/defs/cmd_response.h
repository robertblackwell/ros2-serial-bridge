//
// Created by robert on 12/27/25.
//

#ifndef CPP_SERIAL_BRIDGE_CMD_RESPONSE_H
#define CPP_SERIAL_BRIDGE_CMD_RESPONSE_H
#include <string>
struct CmdResponse {
    std::string status;
    std::string description;
};
struct EncoderStatus {

};
 struct TwoEncoderStatus {
 };

const auto EchoCmdTag = "e";
struct EchoCmd {
    std::string id;
    std::string text;
};

struct LoadTestCmd {

};

const auto MotorPwmCmdTag = "pwm";
struct MotorPwmCmd {

};

const auto MotorRpmCmdTag = "rpm";
struct MotorRpmCmd {

};

const auto ReadEncodersCmdTag = "e";
struct ReadEncodersCmd {

};

struct TextMessage {

};
#endif //CPP_SERIAL_BRIDGE_CMD_RESPONSE_H