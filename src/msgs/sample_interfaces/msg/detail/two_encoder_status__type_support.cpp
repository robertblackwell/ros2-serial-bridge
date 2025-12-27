// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sample_interfaces:msg/TwoEncoderStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sample_interfaces/msg/detail/two_encoder_status__functions.h"
#include "sample_interfaces/msg/detail/two_encoder_status__struct.hpp"
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

void TwoEncoderStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sample_interfaces::msg::TwoEncoderStatus(_init);
}

void TwoEncoderStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sample_interfaces::msg::TwoEncoderStatus *>(message_memory);
  typed_message->~TwoEncoderStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TwoEncoderStatus_message_member_array[2] = {
  {
    "left",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sample_interfaces::msg::EncoderStatus>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::TwoEncoderStatus, left),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sample_interfaces::msg::EncoderStatus>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::TwoEncoderStatus, right),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TwoEncoderStatus_message_members = {
  "sample_interfaces::msg",  // message namespace
  "TwoEncoderStatus",  // message name
  2,  // number of fields
  sizeof(sample_interfaces::msg::TwoEncoderStatus),
  TwoEncoderStatus_message_member_array,  // message members
  TwoEncoderStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  TwoEncoderStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TwoEncoderStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TwoEncoderStatus_message_members,
  get_message_typesupport_handle_function,
  &sample_interfaces__msg__TwoEncoderStatus__get_type_hash,
  &sample_interfaces__msg__TwoEncoderStatus__get_type_description,
  &sample_interfaces__msg__TwoEncoderStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sample_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sample_interfaces::msg::TwoEncoderStatus>()
{
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::TwoEncoderStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sample_interfaces, msg, TwoEncoderStatus)() {
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::TwoEncoderStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
