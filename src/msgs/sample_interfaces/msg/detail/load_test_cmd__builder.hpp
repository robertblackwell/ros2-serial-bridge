// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sample_interfaces:msg/LoadTestCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__BUILDER_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sample_interfaces/msg/detail/load_test_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sample_interfaces
{

namespace msg
{

namespace builder
{

class Init_LoadTestCmd_msgs_per_seecond
{
public:
  explicit Init_LoadTestCmd_msgs_per_seecond(::sample_interfaces::msg::LoadTestCmd & msg)
  : msg_(msg)
  {}
  ::sample_interfaces::msg::LoadTestCmd msgs_per_seecond(::sample_interfaces::msg::LoadTestCmd::_msgs_per_seecond_type arg)
  {
    msg_.msgs_per_seecond = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sample_interfaces::msg::LoadTestCmd msg_;
};

class Init_LoadTestCmd_msg_length
{
public:
  explicit Init_LoadTestCmd_msg_length(::sample_interfaces::msg::LoadTestCmd & msg)
  : msg_(msg)
  {}
  Init_LoadTestCmd_msgs_per_seecond msg_length(::sample_interfaces::msg::LoadTestCmd::_msg_length_type arg)
  {
    msg_.msg_length = std::move(arg);
    return Init_LoadTestCmd_msgs_per_seecond(msg_);
  }

private:
  ::sample_interfaces::msg::LoadTestCmd msg_;
};

class Init_LoadTestCmd_count
{
public:
  Init_LoadTestCmd_count()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadTestCmd_msg_length count(::sample_interfaces::msg::LoadTestCmd::_count_type arg)
  {
    msg_.count = std::move(arg);
    return Init_LoadTestCmd_msg_length(msg_);
  }

private:
  ::sample_interfaces::msg::LoadTestCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sample_interfaces::msg::LoadTestCmd>()
{
  return sample_interfaces::msg::builder::Init_LoadTestCmd_count();
}

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__BUILDER_HPP_
