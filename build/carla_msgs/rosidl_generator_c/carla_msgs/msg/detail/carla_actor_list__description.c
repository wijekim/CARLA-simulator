// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaActorList.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_actor_list__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaActorList__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9c, 0x84, 0xc0, 0xe4, 0xb7, 0x51, 0x35, 0x32,
      0xa2, 0x94, 0x3a, 0x25, 0xc7, 0x5d, 0x4a, 0x13,
      0x9d, 0x48, 0xf3, 0x12, 0x88, 0xf5, 0xa2, 0xb6,
      0x8c, 0x28, 0x9f, 0x1e, 0x0a, 0x0b, 0x1b, 0xe1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "carla_msgs/msg/detail/carla_actor_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t carla_msgs__msg__CarlaActorInfo__EXPECTED_HASH = {1, {
    0x2c, 0xa0, 0xa6, 0xe2, 0xb4, 0x12, 0xf1, 0xff,
    0xee, 0xa4, 0x38, 0xbd, 0xe6, 0x68, 0x3b, 0x1b,
    0xe0, 0xe4, 0x6c, 0x52, 0x92, 0x5a, 0xc4, 0x5a,
    0x6f, 0xe9, 0x1f, 0x8c, 0xb3, 0xc4, 0x37, 0x1c,
  }};
#endif

static char carla_msgs__msg__CarlaActorList__TYPE_NAME[] = "carla_msgs/msg/CarlaActorList";
static char carla_msgs__msg__CarlaActorInfo__TYPE_NAME[] = "carla_msgs/msg/CarlaActorInfo";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaActorList__FIELD_NAME__actors[] = "actors";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaActorList__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaActorList__FIELD_NAME__actors, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {carla_msgs__msg__CarlaActorInfo__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription carla_msgs__msg__CarlaActorList__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {carla_msgs__msg__CarlaActorInfo__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaActorList__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaActorList__TYPE_NAME, 29, 29},
      {carla_msgs__msg__CarlaActorList__FIELDS, 1, 1},
    },
    {carla_msgs__msg__CarlaActorList__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&carla_msgs__msg__CarlaActorInfo__EXPECTED_HASH, carla_msgs__msg__CarlaActorInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = carla_msgs__msg__CarlaActorInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#\n"
  "# Copyright (c) 2019 Intel Corporation.\n"
  "#\n"
  "# This work is licensed under the terms of the MIT license.\n"
  "# For a copy, see <https://opensource.org/licenses/MIT>.\n"
  "#\n"
  "\n"
  "CarlaActorInfo[] actors\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaActorList__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaActorList__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 189, 189},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaActorList__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaActorList__get_individual_type_description_source(NULL),
    sources[1] = *carla_msgs__msg__CarlaActorInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
