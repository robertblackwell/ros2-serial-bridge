// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interfaces:msg/EncoderStatus.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__BUILDER_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interfaces/msg/detail/encoder_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interfaces
{

namespace msg
{

namespace builder
{

class Init_EncoderStatus_direction_pin_state
{
public:
  explicit Init_EncoderStatus_direction_pin_state(::sample_interfaces::msg::EncoderStatus & msg)
  : msg_(msg)
  {}
  ::sample_interfaces::msg::EncoderStatus direction_pin_state(::sample_interfaces::msg::EncoderStatus::_direction_pin_state_type arg)
  {
    msg_.direction_pin_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interfaces::msg::EncoderStatus msg_;
};

class Init_EncoderStatus_motor_rpm_estimate
{
public:
  explicit Init_EncoderStatus_motor_rpm_estimate(::sample_interfaces::msg::EncoderStatus & msg)
  : msg_(msg)
  {}
  Init_EncoderStatus_direction_pin_state motor_rpm_estimate(::sample_interfaces::msg::EncoderStatus::_motor_rpm_estimate_type arg)
  {
    msg_.motor_rpm_estimate = std::move(arg);
    return Init_EncoderStatus_direction_pin_state(msg_);
  }

private:
  ::sample_interfaces::msg::EncoderStatus msg_;
};

class Init_EncoderStatus_sample_time_stamp_usecs
{
public:
  explicit Init_EncoderStatus_sample_time_stamp_usecs(::sample_interfaces::msg::EncoderStatus & msg)
  : msg_(msg)
  {}
  Init_EncoderStatus_motor_rpm_estimate sample_time_stamp_usecs(::sample_interfaces::msg::EncoderStatus::_sample_time_stamp_usecs_type arg)
  {
    msg_.sample_time_stamp_usecs = std::move(arg);
    return Init_EncoderStatus_motor_rpm_estimate(msg_);
  }

private:
  ::sample_interfaces::msg::EncoderStatus msg_;
};

class Init_EncoderStatus_sample_sum
{
public:
  Init_EncoderStatus_sample_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderStatus_sample_time_stamp_usecs sample_sum(::sample_interfaces::msg::EncoderStatus::_sample_sum_type arg)
  {
    msg_.sample_sum = std::move(arg);
    return Init_EncoderStatus_sample_time_stamp_usecs(msg_);
  }

private:
  ::sample_interfaces::msg::EncoderStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interfaces::msg::EncoderStatus>()
{
  return sample_interfaces::msg::builder::Init_EncoderStatus_sample_sum();
}

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__BUILDER_HPP_
