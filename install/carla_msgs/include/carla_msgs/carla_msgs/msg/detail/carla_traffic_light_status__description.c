// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaTrafficLightStatus.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_traffic_light_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaTrafficLightStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe4, 0xa5, 0x8f, 0x69, 0x56, 0xc9, 0xd3, 0x95,
      0xf8, 0xaa, 0x8d, 0xf3, 0x3b, 0x6a, 0xa2, 0x76,
      0x5c, 0xb8, 0x3c, 0x85, 0x22, 0xec, 0x3e, 0x67,
      0x61, 0x96, 0xdc, 0x49, 0x48, 0x5b, 0xf0, 0xb8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char carla_msgs__msg__CarlaTrafficLightStatus__TYPE_NAME[] = "carla_msgs/msg/CarlaTrafficLightStatus";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaTrafficLightStatus__FIELD_NAME__id[] = "id";
static char carla_msgs__msg__CarlaTrafficLightStatus__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaTrafficLightStatus__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaTrafficLightStatus__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaTrafficLightStatus__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaTrafficLightStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaTrafficLightStatus__TYPE_NAME, 38, 38},
      {carla_msgs__msg__CarlaTrafficLightStatus__FIELDS, 2, 2},
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
  "# Copyright (c) 2020 Intel Corporation.\n"
  "#\n"
  "# This work is licensed under the terms of the MIT license.\n"
  "# For a copy, see <https://opensource.org/licenses/MIT>.\n"
  "#\n"
  "uint32 id\n"
  "\n"
  "uint8 RED=0\n"
  "uint8 YELLOW=1\n"
  "uint8 GREEN=2\n"
  "uint8 OFF=3\n"
  "uint8 UNKNOWN=4\n"
  "\n"
  "uint8 state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaTrafficLightStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaTrafficLightStatus__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 256, 256},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaTrafficLightStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaTrafficLightStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
