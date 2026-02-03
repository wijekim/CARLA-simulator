// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaWorldInfo.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_world_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaWorldInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb7, 0x09, 0x3c, 0x2a, 0x34, 0x07, 0x8b, 0x68,
      0x2b, 0xef, 0x09, 0xc0, 0xe6, 0x0c, 0x76, 0xb0,
      0x65, 0x42, 0xf8, 0xab, 0x4e, 0xa6, 0x97, 0x56,
      0xf9, 0x50, 0x5c, 0xa0, 0x66, 0x55, 0xe9, 0xab,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char carla_msgs__msg__CarlaWorldInfo__TYPE_NAME[] = "carla_msgs/msg/CarlaWorldInfo";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaWorldInfo__FIELD_NAME__map_name[] = "map_name";
static char carla_msgs__msg__CarlaWorldInfo__FIELD_NAME__opendrive[] = "opendrive";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaWorldInfo__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaWorldInfo__FIELD_NAME__map_name, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWorldInfo__FIELD_NAME__opendrive, 9, 9},
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
carla_msgs__msg__CarlaWorldInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaWorldInfo__TYPE_NAME, 29, 29},
      {carla_msgs__msg__CarlaWorldInfo__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#\n"
  "# Copyright (c) 2018-2019 Intel Corporation.\n"
  "#\n"
  "# This work is licensed under the terms of the MIT license.\n"
  "# For a copy, see <https://opensource.org/licenses/MIT>.\n"
  "#\n"
  "\n"
  "string map_name\n"
  "string opendrive";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaWorldInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaWorldInfo__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 202, 202},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaWorldInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaWorldInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
