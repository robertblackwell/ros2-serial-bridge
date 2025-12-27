// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interfaces:msg/EchoCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__BUILDER_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interfaces/msg/detail/echo_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interfaces
{

namespace msg
{

namespace builder
{

class Init_EchoCmd_data
{
public:
  Init_EchoCmd_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sample_interfaces::msg::EchoCmd data(::sample_interfaces::msg::EchoCmd::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interfaces::msg::EchoCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interfaces::msg::EchoCmd>()
{
  return sample_interfaces::msg::builder::Init_EchoCmd_data();
}

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__BUILDER_HPP_
