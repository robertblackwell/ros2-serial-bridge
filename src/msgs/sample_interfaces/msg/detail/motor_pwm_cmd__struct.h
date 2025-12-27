// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interfaces:msg/MotorPwmCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__STRUCT_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MotorPwmCmd in the package sample_interfaces.
/**
  *  Set the power (pwm) applied to each motor expressed in percentage -100% .. +100%
  * sign of the values indicates direction +ve 'forward' 
 */
typedef struct sample_interfaces__msg__MotorPwmCmd
{
  float left_motor_pwm;
  float right_motor_pwm;
} sample_interfaces__msg__MotorPwmCmd;

// Struct for a sequence of sample_interfaces__msg__MotorPwmCmd.
typedef struct sample_interfaces__msg__MotorPwmCmd__Sequence
{
  sample_interfaces__msg__MotorPwmCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interfaces__msg__MotorPwmCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__STRUCT_H_
