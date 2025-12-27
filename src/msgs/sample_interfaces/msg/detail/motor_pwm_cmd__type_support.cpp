// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sample_interfaces:msg/MotorPwmCmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sample_interfaces/msg/detail/motor_pwm_cmd__functions.h"
#include "sample_interfaces/msg/detail/motor_pwm_cmd__struct.hpp"
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

void MotorPwmCmd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sample_interfaces::msg::MotorPwmCmd(_init);
}

void MotorPwmCmd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sample_interfaces::msg::MotorPwmCmd *>(message_memory);
  typed_message->~MotorPwmCmd();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorPwmCmd_message_member_array[2] = {
  {
    "left_motor_pwm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::MotorPwmCmd, left_motor_pwm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_motor_pwm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sample_interfaces::msg::MotorPwmCmd, right_motor_pwm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorPwmCmd_message_members = {
  "sample_interfaces::msg",  // message namespace
  "MotorPwmCmd",  // message name
  2,  // number of fields
  sizeof(sample_interfaces::msg::MotorPwmCmd),
  MotorPwmCmd_message_member_array,  // message members
  MotorPwmCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorPwmCmd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorPwmCmd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorPwmCmd_message_members,
  get_message_typesupport_handle_function,
  &sample_interfaces__msg__MotorPwmCmd__get_type_hash,
  &sample_interfaces__msg__MotorPwmCmd__get_type_description,
  &sample_interfaces__msg__MotorPwmCmd__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sample_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sample_interfaces::msg::MotorPwmCmd>()
{
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorPwmCmd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sample_interfaces, msg, MotorPwmCmd)() {
  return &::sample_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorPwmCmd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
