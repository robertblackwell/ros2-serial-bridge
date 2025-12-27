// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/MotorRpmCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MotorRpmCmd in the package sample_interfaces.
/**
  *  Set the speed of the motors to this many revs per second
  * sign of the values indicates direction +ve 'forward' 
 */
typedef struct sample_interfaces__msg__MotorRpmCmd
{
  float left_motor;
  float right_motor;
} sample_interfaces__msg__MotorRpmCmd;

// Struct for a sequence of sample_interfaces__msg__MotorRpmCmd.
typedef struct sample_interfaces__msg__MotorRpmCmd__Sequence
{
  sample_interfaces__msg__MotorRpmCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__MotorRpmCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__STRUCT_H_
