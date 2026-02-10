// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaStatus.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa3, 0xf7, 0xfb, 0x6f, 0xc4, 0xb6, 0x73, 0x84,
      0x0e, 0x0c, 0x09, 0x2c, 0x3c, 0x9e, 0x52, 0xa8,
      0x12, 0x08, 0x85, 0xba, 0x60, 0x3e, 0x01, 0x25,
      0xa2, 0x3f, 0x58, 0xac, 0x27, 0x9c, 0x0c, 0x13,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char carla_msgs__msg__CarlaStatus__TYPE_NAME[] = "carla_msgs/msg/CarlaStatus";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaStatus__FIELD_NAME__frame[] = "frame";
static char carla_msgs__msg__CarlaStatus__FIELD_NAME__fixed_delta_seconds[] = "fixed_delta_seconds";
static char carla_msgs__msg__CarlaStatus__FIELD_NAME__synchronous_mode[] = "synchronous_mode";
static char carla_msgs__msg__CarlaStatus__FIELD_NAME__synchronous_mode_running[] = "synchronous_mode_running";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaStatus__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaStatus__FIELD_NAME__frame, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaStatus__FIELD_NAME__fixed_delta_seconds, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaStatus__FIELD_NAME__synchronous_mode, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaStatus__FIELD_NAME__synchronous_mode_running, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaStatus__TYPE_NAME, 26, 26},
      {carla_msgs__msg__CarlaStatus__FIELDS, 4, 4},
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
  "uint64 frame                  # frame number\n"
  "\n"
  "float32 fixed_delta_seconds   # duration of one frame\n"
  "bool synchronous_mode         # carla is in synchronous mode\n"
  "bool synchronous_mode_running # true: running, false: paused";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaStatus__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 385, 385},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
