// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interfaces:msg/TextMsg.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__TEXT_MSG__BUILDER_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__TEXT_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interfaces/msg/detail/text_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interfaces
{

namespace msg
{

namespace builder
{

class Init_TextMsg_text
{
public:
  Init_TextMsg_text()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sample_interfaces::msg::TextMsg text(::sample_interfaces::msg::TextMsg::_text_type arg)
  {
    msg_.text = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interfaces::msg::TextMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interfaces::msg::TextMsg>()
{
  return sample_interfaces::msg::builder::Init_TextMsg_text();
}

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__TEXT_MSG__BUILDER_HPP_
