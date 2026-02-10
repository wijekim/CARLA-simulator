// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaEgoVehicleControl.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_ego_vehicle_control__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaEgoVehicleControl__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4f, 0x25, 0x1f, 0xa2, 0xa5, 0x54, 0xe8, 0xed,
      0x99, 0x6f, 0x77, 0xeb, 0x1d, 0x5b, 0x65, 0x51,
      0x5a, 0xf1, 0x36, 0x9e, 0xce, 0xb0, 0x4d, 0x51,
      0x22, 0xcb, 0x57, 0x61, 0xf7, 0x80, 0x1b, 0xe3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char carla_msgs__msg__CarlaEgoVehicleControl__TYPE_NAME[] = "carla_msgs/msg/CarlaEgoVehicleControl";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__header[] = "header";
static char carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__throttle[] = "throttle";
static char carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__steer[] = "steer";
static char carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__brake[] = "brake";
static char carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__hand_brake[] = "hand_brake";
static char carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__reverse[] = "reverse";
static char carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__gear[] = "gear";
static char carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__manual_gear_shift[] = "manual_gear_shift";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaEgoVehicleControl__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__throttle, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__steer, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__brake, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__hand_brake, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__reverse, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__gear, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleControl__FIELD_NAME__manual_gear_shift, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription carla_msgs__msg__CarlaEgoVehicleControl__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaEgoVehicleControl__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaEgoVehicleControl__TYPE_NAME, 37, 37},
      {carla_msgs__msg__CarlaEgoVehicleControl__FIELDS, 8, 8},
    },
    {carla_msgs__msg__CarlaEgoVehicleControl__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
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
  "# This represents a vehicle control message sent to CARLA simulator\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# The CARLA vehicle control data\n"
  "\n"
  "# 0. <= throttle <= 1.\n"
  "float32 throttle\n"
  "\n"
  "# -1. <= steer <= 1.\n"
  "float32 steer\n"
  "\n"
  "# 0. <= brake <= 1.\n"
  "float32 brake\n"
  "\n"
  "# hand_brake 0 or 1\n"
  "bool hand_brake\n"
  "\n"
  "# reverse 0 or 1\n"
  "bool reverse\n"
  "\n"
  "# gear\n"
  "int32 gear\n"
  "\n"
  "# manual gear shift\n"
  "bool manual_gear_shift";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaEgoVehicleControl__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaEgoVehicleControl__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 537, 537},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaEgoVehicleControl__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaEgoVehicleControl__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
