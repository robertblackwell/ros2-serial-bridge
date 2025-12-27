// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/LoadTestCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/LoadTestCmd in the package sample_interfaces.
/**
  * Instructs a mucontroller to send data to host as a load test
 */
typedef struct sample_interfaces__msg__LoadTestCmd
{
  /// number of messages to send in a single test cycle
  int32_t count;
  /// make each message approximately this long in bytes
  int32_t msg_length;
  /// send this many messages per second
  int32_t msgs_per_seecond;
} sample_interfaces__msg__LoadTestCmd;

// Struct for a sequence of sample_interfaces__msg__LoadTestCmd.
typedef struct sample_interfaces__msg__LoadTestCmd__Sequence
{
  sample_interfaces__msg__LoadTestCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__LoadTestCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__STRUCT_H_
