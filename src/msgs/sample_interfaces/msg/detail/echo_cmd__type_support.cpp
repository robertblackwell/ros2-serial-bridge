// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sample_interfaces:msg/EchoCmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sample_interfaces/msg/detail/echo_cmd__functions.h"
#include "sample_interfaces/msg/detail/echo_cmd__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace sample_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void EchoCmd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sample_interfaces::msg::EchoCmd(_init);
}

void EchoCmd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sample_interfaces::msg::EchoCmd *>(message_memory);
  typed_message->~EchoCmd();
}

size_t size_function__EchoCmd__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EchoCmd__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__EchoCmd__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__EchoCmd__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__EchoCmd__data(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__EchoCmd__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__EchoCmd__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__EchoCmd__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EchoCmd_message_member_array[1] = {
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::EchoCmd, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__EchoCmd__data,  // size() function pointer
    get_const_function__EchoCmd__data,  // get_const(index) function pointer
    get_function__EchoCmd__data,  // get(index) function pointer
    fetch_function__EchoCmd__data,  // fetch(index, &value) function pointer
    assign_function__EchoCmd__data,  // assign(index, value) function pointer
    resize_function__EchoCmd__data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EchoCmd_message_members = {
  "sample_interfaces::msg",  // message namespace
  "EchoCmd",  // message name
  1,  // number of fields
  sizeof(sample_interfaces::msg::EchoCmd),
  EchoCmd_message_member_array,  // message members
  EchoCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  EchoCmd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EchoCmd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EchoCmd_message_members,
  get_message_typesupport_handle_function,
  &sample_interfaces__msg__EchoCmd__get_type_hash,
  &sample_interfaces__msg__EchoCmd__get_type_description,
  &sample_interfaces__msg__EchoCmd__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sample_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sample_interfaces::msg::EchoCmd>()
{
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::EchoCmd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sample_interfaces, msg, EchoCmd)() {
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::EchoCmd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
