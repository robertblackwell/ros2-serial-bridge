// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/EncoderStatus.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/EncoderStatus in the package sample_interfaces.
/**
  * Reports the current status of a single motor encoder
 */
typedef struct sample_interfaces__msg__EncoderStatus
{
  int64_t sample_sum;
  int64_t sample_time_stamp_usecs;
  float motor_rpm_estimate;
  uint8_t direction_pin_state;
} sample_interfaces__msg__EncoderStatus;

// Struct for a sequence of sample_interfaces__msg__EncoderStatus.
typedef struct sample_interfaces__msg__EncoderStatus__Sequence
{
  sample_interfaces__msg__EncoderStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__EncoderStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__STRUCT_H_
