// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaActorInfo.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_actor_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaActorInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2c, 0xa0, 0xa6, 0xe2, 0xb4, 0x12, 0xf1, 0xff,
      0xee, 0xa4, 0x38, 0xbd, 0xe6, 0x68, 0x3b, 0x1b,
      0xe0, 0xe4, 0x6c, 0x52, 0x92, 0x5a, 0xc4, 0x5a,
      0x6f, 0xe9, 0x1f, 0x8c, 0xb3, 0xc4, 0x37, 0x1c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char carla_msgs__msg__CarlaActorInfo__TYPE_NAME[] = "carla_msgs/msg/CarlaActorInfo";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaActorInfo__FIELD_NAME__id[] = "id";
static char carla_msgs__msg__CarlaActorInfo__FIELD_NAME__parent_id[] = "parent_id";
static char carla_msgs__msg__CarlaActorInfo__FIELD_NAME__type[] = "type";
static char carla_msgs__msg__CarlaActorInfo__FIELD_NAME__rolename[] = "rolename";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaActorInfo__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaActorInfo__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaActorInfo__FIELD_NAME__parent_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaActorInfo__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaActorInfo__FIELD_NAME__rolename, 8, 8},
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
carla_msgs__msg__CarlaActorInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaActorInfo__TYPE_NAME, 29, 29},
      {carla_msgs__msg__CarlaActorInfo__FIELDS, 4, 4},
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
  "# Copyright (c) 2019 Intel Corporation.\n"
  "#\n"
  "# This work is licensed under the terms of the MIT license.\n"
  "# For a copy, see <https://opensource.org/licenses/MIT>.\n"
  "#\n"
  "\n"
  "uint32 id\n"
  "uint32 parent_id # 0 if no parent available\n"
  "string type\n"
  "string rolename\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaActorInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaActorInfo__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 247, 247},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaActorInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaActorInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
