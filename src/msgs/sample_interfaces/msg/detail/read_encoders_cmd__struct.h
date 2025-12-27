// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/ReadEncodersCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/ReadEncodersCmd in the package sample_interfaces.
/**
  * Tells micro controller to output 'n' TwoEncoderStatus messages 
  * Current implementation of the firmware ignores the argument 'n' and only outputs 1 status message.
 */
typedef struct sample_interfaces__msg__ReadEncodersCmd
{
  int32_t n;
} sample_interfaces__msg__ReadEncodersCmd;

// Struct for a sequence of sample_interfaces__msg__ReadEncodersCmd.
typedef struct sample_interfaces__msg__ReadEncodersCmd__Sequence
{
  sample_interfaces__msg__ReadEncodersCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__ReadEncodersCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__STRUCT_H_
