// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/MotorPwmCmd.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/motor_pwm_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__MotorPwmCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xac, 0xba, 0x7d, 0x0a, 0xd3, 0x7f, 0x44, 0x45,
      0xd8, 0x7b, 0xa9, 0x1c, 0x2e, 0xc3, 0xf2, 0x02,
      0x5c, 0x64, 0x55, 0x63, 0x52, 0x21, 0x39, 0x1d,
      0xe8, 0x48, 0xce, 0x1a, 0xcb, 0x83, 0xf4, 0x9c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_interfaces__msg__MotorPwmCmd__TYPE_NAME[] = "sample_interfaces/msg/MotorPwmCmd";

// Define type names, field names, and default values
static char sample_interfaces__msg__MotorPwmCmd__FIELD_NAME__left_motor_pwm[] = "left_motor_pwm";
static char sample_interfaces__msg__MotorPwmCmd__FIELD_NAME__right_motor_pwm[] = "right_motor_pwm";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__MotorPwmCmd__FIELDS[] = {
  {
    {sample_interfaces__msg__MotorPwmCmd__FIELD_NAME__left_motor_pwm, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__MotorPwmCmd__FIELD_NAME__right_motor_pwm, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_interfaces__msg__MotorPwmCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__MotorPwmCmd__TYPE_NAME, 33, 33},
      {sample_interfaces__msg__MotorPwmCmd__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Set the power (pwm) applied to each motor expressed in percentage -100% .. +100%\n"
  "#sign of the values indicates direction +ve 'forward' \n"
  "float32 left_motor_pwm\n"
  "float32 right_motor_pwm";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__MotorPwmCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__MotorPwmCmd__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 184, 184},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__MotorPwmCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__MotorPwmCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
