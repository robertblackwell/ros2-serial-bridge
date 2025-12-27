// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sample_interfaces:msg/TwoEncoderStatus.idl
// generated code does not contain a copyright notice

#include "sample_interfaces/msg/detail/two_encoder_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__TwoEncoderStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5d, 0x3b, 0x2b, 0xe0, 0x25, 0x47, 0xc9, 0xd5,
      0x10, 0x42, 0x7e, 0x19, 0x25, 0x36, 0xba, 0x37,
      0x39, 0x6c, 0xaa, 0xb2, 0x92, 0x1b, 0x28, 0x8b,
      0x44, 0x78, 0xa4, 0x15, 0xea, 0x30, 0x88, 0x88,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "sample_interfaces/msg/detail/encoder_status__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t sample_interfaces__msg__EncoderStatus__EXPECTED_HASH = {1, {
    0x0e, 0x4d, 0xea, 0xf8, 0x9d, 0xc5, 0x85, 0x30,
    0xc1, 0x2a, 0xfb, 0xb2, 0x0c, 0x9d, 0x0e, 0x3e,
    0x97, 0x67, 0xc0, 0x2e, 0x6b, 0x4a, 0xaf, 0xd9,
    0x38, 0xf2, 0x80, 0x88, 0x9f, 0xee, 0xc4, 0x4f,
  }};
#endif

static char sample_interfaces__msg__TwoEncoderStatus__TYPE_NAME[] = "sample_interfaces/msg/TwoEncoderStatus";
static char sample_interfaces__msg__EncoderStatus__TYPE_NAME[] = "sample_interfaces/msg/EncoderStatus";

// Define type names, field names, and default values
static char sample_interfaces__msg__TwoEncoderStatus__FIELD_NAME__left[] = "left";
static char sample_interfaces__msg__TwoEncoderStatus__FIELD_NAME__right[] = "right";

static rosidl_runtime_c__type_description__Field sample_interfaces__msg__TwoEncoderStatus__FIELDS[] = {
  {
    {sample_interfaces__msg__TwoEncoderStatus__FIELD_NAME__left, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sample_interfaces__msg__EncoderStatus__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {sample_interfaces__msg__TwoEncoderStatus__FIELD_NAME__right, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sample_interfaces__msg__EncoderStatus__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription sample_interfaces__msg__TwoEncoderStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {sample_interfaces__msg__EncoderStatus__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sample_interfaces__msg__TwoEncoderStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sample_interfaces__msg__TwoEncoderStatus__TYPE_NAME, 38, 38},
      {sample_interfaces__msg__TwoEncoderStatus__FIELDS, 2, 2},
    },
    {sample_interfaces__msg__TwoEncoderStatus__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&sample_interfaces__msg__EncoderStatus__EXPECTED_HASH, sample_interfaces__msg__EncoderStatus__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = sample_interfaces__msg__EncoderStatus__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Reports the current status of two motor encoder\n"
  "\n"
  "EncoderStatus  left\n"
  "EncoderStatus  right\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__TwoEncoderStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sample_interfaces__msg__TwoEncoderStatus__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 92, 92},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__TwoEncoderStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sample_interfaces__msg__TwoEncoderStatus__get_individual_type_description_source(NULL),
    sources[1] = *sample_interfaces__msg__EncoderStatus__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
