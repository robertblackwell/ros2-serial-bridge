// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/EchoCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/EchoCmd in the package sample_interfaces.
/**
  * This message will be echoed and can take any number of string arguments
 */
typedef struct sample_interfaces__msg__EchoCmd
{
  rosidl_runtime_c__String__Sequence data;
} sample_interfaces__msg__EchoCmd;

// Struct for a sequence of sample_interfaces__msg__EchoCmd.
typedef struct sample_interfaces__msg__EchoCmd__Sequence
{
  sample_interfaces__msg__EchoCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__EchoCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__STRUCT_H_
