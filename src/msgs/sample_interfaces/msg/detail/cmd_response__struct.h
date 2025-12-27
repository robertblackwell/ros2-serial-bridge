// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/CmdResponse.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__STRUCT_H_

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

/// Struct defined in msg/CmdResponse in the package sample_interfaces.
typedef struct sample_interfaces__msg__CmdResponse
{
  bool ok;
  rosidl_runtime_c__String text;
} sample_interfaces__msg__CmdResponse;

// Struct for a sequence of sample_interfaces__msg__CmdResponse.
typedef struct sample_interfaces__msg__CmdResponse__Sequence
{
  sample_interfaces__msg__CmdResponse * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__CmdResponse__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__STRUCT_H_
