// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sample_interfaces:msg/EncoderStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sample_interfaces/msg/detail/encoder_status__functions.h"
#include "sample_interfaces/msg/detail/encoder_status__struct.hpp"
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

void EncoderStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sample_interfaces::msg::EncoderStatus(_init);
}

void EncoderStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sample_interfaces::msg::EncoderStatus *>(message_memory);
  typed_message->~EncoderStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EncoderStatus_message_member_array[4] = {
  {
    "sample_sum",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::EncoderStatus, sample_sum),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sample_time_stamp_usecs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::EncoderStatus, sample_time_stamp_usecs),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_rpm_estimate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::EncoderStatus, motor_rpm_estimate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "direction_pin_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::EncoderStatus, direction_pin_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EncoderStatus_message_members = {
  "sample_interfaces::msg",  // message namespace
  "EncoderStatus",  // message name
  4,  // number of fields
  sizeof(sample_interfaces::msg::EncoderStatus),
  EncoderStatus_message_member_array,  // message members
  EncoderStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  EncoderStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EncoderStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EncoderStatus_message_members,
  get_message_typesupport_handle_function,
  &sample_interfaces__msg__EncoderStatus__get_type_hash,
  &sample_interfaces__msg__EncoderStatus__get_type_description,
  &sample_interfaces__msg__EncoderStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sample_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sample_interfaces::msg::EncoderStatus>()
{
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::EncoderStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sample_interfaces, msg, EncoderStatus)() {
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::EncoderStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
