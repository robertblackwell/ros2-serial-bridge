// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/EncoderStatus.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/encoder_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__EncoderStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0e, 0x4d, 0xea, 0xf8, 0x9d, 0xc5, 0x85, 0x30,
      0xc1, 0x2a, 0xfb, 0xb2, 0x0c, 0x9d, 0x0e, 0x3e,
      0x97, 0x67, 0xc0, 0x2e, 0x6b, 0x4a, 0xaf, 0xd9,
      0x38, 0xf2, 0x80, 0x88, 0x9f, 0xee, 0xc4, 0x4f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_interfaces__msg__EncoderStatus__TYPE_NAME[] = "sample_interfaces/msg/EncoderStatus";

// Define type names, field names, and default values
static char sample_interfaces__msg__EncoderStatus__FIELD_NAME__sample_sum[] = "sample_sum";
static char sample_interfaces__msg__EncoderStatus__FIELD_NAME__sample_time_stamp_usecs[] = "sample_time_stamp_usecs";
static char sample_interfaces__msg__EncoderStatus__FIELD_NAME__motor_rpm_estimate[] = "motor_rpm_estimate";
static char sample_interfaces__msg__EncoderStatus__FIELD_NAME__direction_pin_state[] = "direction_pin_state";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__EncoderStatus__FIELDS[] = {
  {
    {sample_interfaces__msg__EncoderStatus__FIELD_NAME__sample_sum, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__EncoderStatus__FIELD_NAME__sample_time_stamp_usecs, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__EncoderStatus__FIELD_NAME__motor_rpm_estimate, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__EncoderStatus__FIELD_NAME__direction_pin_state, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_interfaces__msg__EncoderStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__EncoderStatus__TYPE_NAME, 35, 35},
      {sample_interfaces__msg__EncoderStatus__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Reports the current status of a single motor encoder\n"
  "int64    sample_sum\n"
  "int64    sample_time_stamp_usecs\n"
  "float32  motor_rpm_estimate\n"
  "char     direction_pin_state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__EncoderStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__EncoderStatus__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 163, 163},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__EncoderStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__EncoderStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
