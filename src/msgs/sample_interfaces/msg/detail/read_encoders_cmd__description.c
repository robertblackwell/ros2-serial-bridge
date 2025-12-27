// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/ReadEncodersCmd.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/read_encoders_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__ReadEncodersCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x04, 0xfd, 0xa5, 0x5a, 0xd5, 0x2d, 0x9d, 0x2c,
      0xcc, 0xde, 0xf8, 0x5a, 0xed, 0x9e, 0xa2, 0xf8,
      0x15, 0x8d, 0x2e, 0x67, 0x20, 0xd2, 0x57, 0x89,
      0x10, 0x05, 0xce, 0x23, 0x79, 0xf3, 0xf6, 0xac,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_interfaces__msg__ReadEncodersCmd__TYPE_NAME[] = "sample_interfaces/msg/ReadEncodersCmd";

// Define type names, field names, and default values
static char sample_interfaces__msg__ReadEncodersCmd__FIELD_NAME__n[] = "n";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__ReadEncodersCmd__FIELDS[] = {
  {
    {sample_interfaces__msg__ReadEncodersCmd__FIELD_NAME__n, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_interfaces__msg__ReadEncodersCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__ReadEncodersCmd__TYPE_NAME, 37, 37},
      {sample_interfaces__msg__ReadEncodersCmd__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Tells micro controller to output 'n' TwoEncoderStatus messages \n"
  "# Current implementation of the firmware ignores the argument 'n' and only outputs 1 status message.\n"
  "int32  n";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__ReadEncodersCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__ReadEncodersCmd__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 175, 175},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__ReadEncodersCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__ReadEncodersCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
