// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interfaces:msg/MotorRpmCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__BUILDER_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interfaces/msg/detail/motor_rpm_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorRpmCmd_right_motor
{
public:
  explicit Init_MotorRpmCmd_right_motor(::sample_interfaces::msg::MotorRpmCmd & msg)
  : msg_(msg)
  {}
  ::sample_interfaces::msg::MotorRpmCmd right_motor(::sample_interfaces::msg::MotorRpmCmd::_right_motor_type arg)
  {
    msg_.right_motor = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interfaces::msg::MotorRpmCmd msg_;
};

class Init_MotorRpmCmd_left_motor
{
public:
  Init_MotorRpmCmd_left_motor()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorRpmCmd_right_motor left_motor(::sample_interfaces::msg::MotorRpmCmd::_left_motor_type arg)
  {
    msg_.left_motor = std::move(arg);
    return Init_MotorRpmCmd_right_motor(msg_);
  }

private:
  ::sample_interfaces::msg::MotorRpmCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interfaces::msg::MotorRpmCmd>()
{
  return sample_interfaces::msg::builder::Init_MotorRpmCmd_left_motor();
}

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__BUILDER_HPP_
