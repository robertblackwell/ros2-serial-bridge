// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/LoadTestCmd.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/load_test_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__LoadTestCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6e, 0x0e, 0x26, 0xe7, 0xd8, 0x42, 0xcc, 0x3c,
      0x6c, 0xb8, 0x6b, 0x9f, 0xba, 0x4a, 0x4d, 0xc7,
      0x51, 0xc6, 0xf2, 0x96, 0x18, 0xed, 0xff, 0xce,
      0x55, 0xe9, 0xcf, 0xb5, 0xcb, 0xc7, 0xac, 0xce,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sample_interfaces__msg__LoadTestCmd__TYPE_NAME[] = "sample_interfaces/msg/LoadTestCmd";

// Define type names, field names, and default values
static char sample_interfaces__msg__LoadTestCmd__FIELD_NAME__count[] = "count";
static char sample_interfaces__msg__LoadTestCmd__FIELD_NAME__msg_length[] = "msg_length";
static char sample_interfaces__msg__LoadTestCmd__FIELD_NAME__msgs_per_seecond[] = "msgs_per_seecond";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__LoadTestCmd__FIELDS[] = {
  {
    {sample_interfaces__msg__LoadTestCmd__FIELD_NAME__count, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__LoadTestCmd__FIELD_NAME__msg_length, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__LoadTestCmd__FIELD_NAME__msgs_per_seecond, 16, 16},
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
sample_interfaces__msg__LoadTestCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__LoadTestCmd__TYPE_NAME, 33, 33},
      {sample_interfaces__msg__LoadTestCmd__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Instructs a mucontroller to send data to host as a load test\n"
  "int32 count             #number of messages to send in a single test cycle\n"
  "int32 msg_length        #make each message approximately this long in bytes\n"
  "int32 msgs_per_seecond  #send this many messages per second";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__LoadTestCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__LoadTestCmd__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 272, 272},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__LoadTestCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__LoadTestCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
