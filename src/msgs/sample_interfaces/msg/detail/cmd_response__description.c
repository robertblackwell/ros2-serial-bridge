// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/CmdResponse.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/cmd_response__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__CmdResponse__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6b, 0xd9, 0xdc, 0xfd, 0xca, 0xbc, 0x22, 0x28,
      0xf6, 0x62, 0x4d, 0x8e, 0x05, 0xfb, 0x35, 0xe3,
      0x5f, 0x11, 0x7b, 0x6b, 0x98, 0x1f, 0x65, 0xe5,
      0xc8, 0x9d, 0x0c, 0xaa, 0xd2, 0xa8, 0xa0, 0xe9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_interfaces__msg__CmdResponse__TYPE_NAME[] = "sample_interfaces/msg/CmdResponse";

// Define type names, field names, and default values
static char sample_interfaces__msg__CmdResponse__FIELD_NAME__ok[] = "ok";
static char sample_interfaces__msg__CmdResponse__FIELD_NAME__text[] = "text";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__CmdResponse__FIELDS[] = {
  {
    {sample_interfaces__msg__CmdResponse__FIELD_NAME__ok, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__CmdResponse__FIELD_NAME__text, 4, 4},
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
sample_interfaces__msg__CmdResponse__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__CmdResponse__TYPE_NAME, 33, 33},
      {sample_interfaces__msg__CmdResponse__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool    ok\n"
  "string  text";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__CmdResponse__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__CmdResponse__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 23, 23},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__CmdResponse__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__CmdResponse__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
