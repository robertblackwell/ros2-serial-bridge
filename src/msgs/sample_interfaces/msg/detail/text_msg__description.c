// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/TextMsg.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/text_msg__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__TextMsg__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x68, 0x17, 0xa9, 0x17, 0x2d, 0xb1, 0x61, 0x3a,
      0xfb, 0xa2, 0xf8, 0xe6, 0xd8, 0xb5, 0xb3, 0xa6,
      0xdb, 0xe8, 0x5a, 0x5a, 0xb9, 0x44, 0xce, 0x97,
      0x2d, 0x69, 0x59, 0x81, 0x31, 0x7f, 0x6a, 0x4e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_interfaces__msg__TextMsg__TYPE_NAME[] = "sample_interfaces/msg/TextMsg";

// Define type names, field names, and default values
static char sample_interfaces__msg__TextMsg__FIELD_NAME__text[] = "text";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__TextMsg__FIELDS[] = {
  {
    {sample_interfaces__msg__TextMsg__FIELD_NAME__text, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_interfaces__msg__TextMsg__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__TextMsg__TYPE_NAME, 29, 29},
      {sample_interfaces__msg__TextMsg__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string text";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__TextMsg__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__TextMsg__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 11, 11},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__TextMsg__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__TextMsg__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
