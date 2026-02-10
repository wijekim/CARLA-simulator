// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaControl.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_control__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaControl__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x70, 0x53, 0xe4, 0xe9, 0xec, 0x21, 0x22, 0x3b,
      0x8f, 0x3a, 0x67, 0x94, 0x4c, 0xc1, 0x9f, 0xac,
      0x0e, 0x41, 0x97, 0x91, 0xa8, 0x7e, 0x59, 0x43,
      0x8a, 0x56, 0xba, 0x9e, 0xfc, 0xe2, 0xa9, 0xac,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char carla_msgs__msg__CarlaControl__TYPE_NAME[] = "carla_msgs/msg/CarlaControl";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaControl__FIELD_NAME__command[] = "command";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaControl__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaControl__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaControl__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaControl__TYPE_NAME, 27, 27},
      {carla_msgs__msg__CarlaControl__FIELDS, 1, 1},
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
  "int8 PLAY = 0\n"
  "int8 PAUSE = 1\n"
  "int8 STEP_ONCE = 2\n"
  "\n"
  "int8 command\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaControl__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaControl__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 227, 227},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaControl__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaControl__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
