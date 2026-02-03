// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaEgoVehicleInfoWheel.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_ego_vehicle_info_wheel__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaEgoVehicleInfoWheel__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x63, 0xca, 0xae, 0x9f, 0x26, 0x37, 0x75, 0x2b,
      0x93, 0xa6, 0x16, 0xb1, 0x0e, 0x00, 0xd2, 0x1b,
      0xe1, 0x5c, 0xa3, 0x2c, 0xac, 0x53, 0x3e, 0xfd,
      0x35, 0x96, 0x34, 0x9a, 0x5c, 0xf4, 0xd4, 0xfe,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/vector3__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
#endif

static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__TYPE_NAME[] = "carla_msgs/msg/CarlaEgoVehicleInfoWheel";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__tire_friction[] = "tire_friction";
static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__damping_rate[] = "damping_rate";
static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__max_steer_angle[] = "max_steer_angle";
static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__radius[] = "radius";
static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__max_brake_torque[] = "max_brake_torque";
static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__max_handbrake_torque[] = "max_handbrake_torque";
static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__position[] = "position";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__tire_friction, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__damping_rate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__max_steer_angle, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__radius, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__max_brake_torque, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__max_handbrake_torque, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription carla_msgs__msg__CarlaEgoVehicleInfoWheel__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaEgoVehicleInfoWheel__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaEgoVehicleInfoWheel__TYPE_NAME, 39, 39},
      {carla_msgs__msg__CarlaEgoVehicleInfoWheel__FIELDS, 7, 7},
    },
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#\n"
  "# Copyright (c) 2019-2020 Intel Corporation.\n"
  "#\n"
  "# This work is licensed under the terms of the MIT license.\n"
  "# For a copy, see <https://opensource.org/licenses/MIT>.\n"
  "#\n"
  "float32 tire_friction\n"
  "float32 damping_rate\n"
  "float32 max_steer_angle\n"
  "float32 radius\n"
  "float32 max_brake_torque\n"
  "float32 max_handbrake_torque\n"
  "geometry_msgs/Vector3 position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaEgoVehicleInfoWheel__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 335, 335},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaEgoVehicleInfoWheel__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaEgoVehicleInfoWheel__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
