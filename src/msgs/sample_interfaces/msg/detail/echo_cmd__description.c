// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/EchoCmd.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/echo_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__EchoCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf1, 0xcb, 0x3d, 0xe0, 0x92, 0x4c, 0x93, 0xcf,
      0x60, 0x51, 0x4f, 0x55, 0x5f, 0x26, 0x7e, 0x5a,
      0x4a, 0x75, 0x2e, 0x56, 0xbe, 0x90, 0xd3, 0x5f,
      0xc7, 0x2f, 0x8a, 0xf2, 0xa3, 0x03, 0xba, 0xd8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_interfaces__msg__EchoCmd__TYPE_NAME[] = "sample_interfaces/msg/EchoCmd";

// Define type names, field names, and default values
static char sample_interfaces__msg__EchoCmd__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__EchoCmd__FIELDS[] = {
  {
    {sample_interfaces__msg__EchoCmd__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_interfaces__msg__EchoCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__EchoCmd__TYPE_NAME, 29, 29},
      {sample_interfaces__msg__EchoCmd__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message will be echoed and can take any number of string arguments\n"
  "\n"
  "string[]            data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__EchoCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__EchoCmd__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 99, 99},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__EchoCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__EchoCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
