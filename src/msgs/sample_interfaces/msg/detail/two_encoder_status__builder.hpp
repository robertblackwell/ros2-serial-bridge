// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interfaces:msg/TwoEncoderStatus.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__BUILDER_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interfaces/msg/detail/two_encoder_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interfaces
{

namespace msg
{

namespace builder
{

class Init_TwoEncoderStatus_right
{
public:
  explicit Init_TwoEncoderStatus_right(::sample_interfaces::msg::TwoEncoderStatus & msg)
  : msg_(msg)
  {}
  ::sample_interfaces::msg::TwoEncoderStatus right(::sample_interfaces::msg::TwoEncoderStatus::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interfaces::msg::TwoEncoderStatus msg_;
};

class Init_TwoEncoderStatus_left
{
public:
  Init_TwoEncoderStatus_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TwoEncoderStatus_right left(::sample_interfaces::msg::TwoEncoderStatus::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_TwoEncoderStatus_right(msg_);
  }

private:
  ::sample_interfaces::msg::TwoEncoderStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interfaces::msg::TwoEncoderStatus>()
{
  return sample_interfaces::msg::builder::Init_TwoEncoderStatus_left();
}

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__BUILDER_HPP_
