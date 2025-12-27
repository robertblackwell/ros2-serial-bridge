// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sample_interfaces:msg/CmdResponse.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sample_interfaces/msg/detail/cmd_response__functions.h"
#include "sample_interfaces/msg/detail/cmd_response__struct.hpp"
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

void CmdResponse_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sample_interfaces::msg::CmdResponse(_init);
}

void CmdResponse_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sample_interfaces::msg::CmdResponse *>(message_memory);
  typed_message->~CmdResponse();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CmdResponse_message_member_array[2] = {
  {
    "ok",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::CmdResponse, ok),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "text",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::CmdResponse, text),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CmdResponse_message_members = {
  "sample_interfaces::msg",  // message namespace
  "CmdResponse",  // message name
  2,  // number of fields
  sizeof(sample_interfaces::msg::CmdResponse),
  CmdResponse_message_member_array,  // message members
  CmdResponse_init_function,  // function to initialize message memory (memory has to be allocated)
  CmdResponse_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CmdResponse_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CmdResponse_message_members,
  get_message_typesupport_handle_function,
  &sample_interfaces__msg__CmdResponse__get_type_hash,
  &sample_interfaces__msg__CmdResponse__get_type_description,
  &sample_interfaces__msg__CmdResponse__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sample_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sample_interfaces::msg::CmdResponse>()
{
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::CmdResponse_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sample_interfaces, msg, CmdResponse)() {
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::CmdResponse_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
