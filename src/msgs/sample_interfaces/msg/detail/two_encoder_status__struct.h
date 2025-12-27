// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/TwoEncoderStatus.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'left'
// Member 'right'
#include "sample_interfaces/msg/detail/encoder_status__struct.h"

/// Struct defined in msg/TwoEncoderStatus in the package sample_interfaces.
/**
  * Reports the current status of two motor encoder
 */
typedef struct sample_interfaces__msg__TwoEncoderStatus
{
  sample_interfaces__msg__EncoderStatus left;
  sample_interfaces__msg__EncoderStatus right;
} sample_interfaces__msg__TwoEncoderStatus;

// Struct for a sequence of sample_interfaces__msg__TwoEncoderStatus.
typedef struct sample_interfaces__msg__TwoEncoderStatus__Sequence
{
  sample_interfaces__msg__TwoEncoderStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__TwoEncoderStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__STRUCT_H_
