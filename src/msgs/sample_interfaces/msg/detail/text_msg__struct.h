// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/TextMsg.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__TEXT_MSG__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__TEXT_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'text'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TextMsg in the package sample_interfaces.
typedef struct sample_interfaces__msg__TextMsg
{
  rosidl_runtime_c__String text;
} sample_interfaces__msg__TextMsg;

// Struct for a sequence of sample_interfaces__msg__TextMsg.
typedef struct sample_interfaces__msg__TextMsg__Sequence
{
  sample_interfaces__msg__TextMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__TextMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__TEXT_MSG__STRUCT_H_
