// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interfaces:msg/CmdResponse.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__BUILDER_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interfaces/msg/detail/cmd_response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interfaces
{

namespace msg
{

namespace builder
{

class Init_CmdResponse_text
{
public:
  explicit Init_CmdResponse_text(::sample_interfaces::msg::CmdResponse & msg)
  : msg_(msg)
  {}
  ::sample_interfaces::msg::CmdResponse text(::sample_interfaces::msg::CmdResponse::_text_type arg)
  {
    msg_.text = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interfaces::msg::CmdResponse msg_;
};

class Init_CmdResponse_ok
{
public:
  Init_CmdResponse_ok()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CmdResponse_text ok(::sample_interfaces::msg::CmdResponse::_ok_type arg)
  {
    msg_.ok = std::move(arg);
    return Init_CmdResponse_text(msg_);
  }

private:
  ::sample_interfaces::msg::CmdResponse msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interfaces::msg::CmdResponse>()
{
  return sample_interfaces::msg::builder::Init_CmdResponse_ok();
}

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__BUILDER_HPP_
