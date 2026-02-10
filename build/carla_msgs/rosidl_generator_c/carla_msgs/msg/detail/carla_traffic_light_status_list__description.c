// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaTrafficLightStatusList.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_traffic_light_status_list__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaTrafficLightStatusList__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x07, 0xf3, 0xb9, 0xec, 0x43, 0x6d, 0x99, 0x25,
      0xe8, 0xb5, 0x5f, 0x22, 0x9d, 0x42, 0x03, 0x36,
      0xa9, 0x97, 0xe8, 0x24, 0x91, 0x1a, 0xfd, 0xb2,
      0x99, 0xf2, 0xdd, 0x0f, 0x32, 0xa2, 0x62, 0x68,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "carla_msgs/msg/detail/carla_traffic_light_status__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t carla_msgs__msg__CarlaTrafficLightStatus__EXPECTED_HASH = {1, {
    0xe4, 0xa5, 0x8f, 0x69, 0x56, 0xc9, 0xd3, 0x95,
    0xf8, 0xaa, 0x8d, 0xf3, 0x3b, 0x6a, 0xa2, 0x76,
    0x5c, 0xb8, 0x3c, 0x85, 0x22, 0xec, 0x3e, 0x67,
    0x61, 0x96, 0xdc, 0x49, 0x48, 0x5b, 0xf0, 0xb8,
  }};
#endif

static char carla_msgs__msg__CarlaTrafficLightStatusList__TYPE_NAME[] = "carla_msgs/msg/CarlaTrafficLightStatusList";
static char carla_msgs__msg__CarlaTrafficLightStatus__TYPE_NAME[] = "carla_msgs/msg/CarlaTrafficLightStatus";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaTrafficLightStatusList__FIELD_NAME__traffic_lights[] = "traffic_lights";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaTrafficLightStatusList__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaTrafficLightStatusList__FIELD_NAME__traffic_lights, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {carla_msgs__msg__CarlaTrafficLightStatus__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription carla_msgs__msg__CarlaTrafficLightStatusList__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {carla_msgs__msg__CarlaTrafficLightStatus__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaTrafficLightStatusList__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaTrafficLightStatusList__TYPE_NAME, 42, 42},
      {carla_msgs__msg__CarlaTrafficLightStatusList__FIELDS, 1, 1},
    },
    {carla_msgs__msg__CarlaTrafficLightStatusList__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&carla_msgs__msg__CarlaTrafficLightStatus__EXPECTED_HASH, carla_msgs__msg__CarlaTrafficLightStatus__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = carla_msgs__msg__CarlaTrafficLightStatus__get_type_description(NULL)->type_description.fields;
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
  "CarlaTrafficLightStatus[] traffic_lights";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaTrafficLightStatusList__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaTrafficLightStatusList__TYPE_NAME, 42, 42},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 204, 204},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaTrafficLightStatusList__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaTrafficLightStatusList__get_individual_type_description_source(NULL),
    sources[1] = *carla_msgs__msg__CarlaTrafficLightStatus__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
