// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/MotorRpmCmd.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/motor_rpm_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__MotorRpmCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x36, 0x8b, 0x5e, 0x53, 0x0e, 0x62, 0x6c, 0x9c,
      0xc2, 0x20, 0x8a, 0xcf, 0xce, 0x36, 0x59, 0x4d,
      0x84, 0x02, 0x88, 0xbe, 0x21, 0x12, 0x3d, 0x52,
      0x0d, 0xca, 0xf7, 0x52, 0x99, 0x49, 0x3e, 0x56,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_interfaces__msg__MotorRpmCmd__TYPE_NAME[] = "sample_interfaces/msg/MotorRpmCmd";

// Define type names, field names, and default values
static char sample_interfaces__msg__MotorRpmCmd__FIELD_NAME__left_motor[] = "left_motor";
static char sample_interfaces__msg__MotorRpmCmd__FIELD_NAME__right_motor[] = "right_motor";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__MotorRpmCmd__FIELDS[] = {
  {
    {sample_interfaces__msg__MotorRpmCmd__FIELD_NAME__left_motor, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__MotorRpmCmd__FIELD_NAME__right_motor, 11, 11},
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
sample_interfaces__msg__MotorRpmCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__MotorRpmCmd__TYPE_NAME, 33, 33},
      {sample_interfaces__msg__MotorRpmCmd__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Set the speed of the motors to this many revs per second\n"
  "#sign of the values indicates direction +ve 'forward' \n"
  "float32 left_motor\n"
  "float32 right_motor";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__MotorRpmCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__MotorRpmCmd__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 152, 152},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__MotorRpmCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__MotorRpmCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
