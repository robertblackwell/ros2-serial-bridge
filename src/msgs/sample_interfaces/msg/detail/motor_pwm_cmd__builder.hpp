// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interfaces:msg/MotorPwmCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__BUILDER_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interfaces/msg/detail/motor_pwm_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorPwmCmd_right_motor_pwm
{
public:
  explicit Init_MotorPwmCmd_right_motor_pwm(::sample_interfaces::msg::MotorPwmCmd & msg)
  : msg_(msg)
  {}
  ::sample_interfaces::msg::MotorPwmCmd right_motor_pwm(::sample_interfaces::msg::MotorPwmCmd::_right_motor_pwm_type arg)
  {
    msg_.right_motor_pwm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interfaces::msg::MotorPwmCmd msg_;
};

class Init_MotorPwmCmd_left_motor_pwm
{
public:
  Init_MotorPwmCmd_left_motor_pwm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorPwmCmd_right_motor_pwm left_motor_pwm(::sample_interfaces::msg::MotorPwmCmd::_left_motor_pwm_type arg)
  {
    msg_.left_motor_pwm = std::move(arg);
    return Init_MotorPwmCmd_right_motor_pwm(msg_);
  }

private:
  ::sample_interfaces::msg::MotorPwmCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interfaces::msg::MotorPwmCmd>()
{
  return sample_interfaces::msg::builder::Init_MotorPwmCmd_left_motor_pwm();
}

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__BUILDER_HPP_
