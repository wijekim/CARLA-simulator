// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaEgoVehicleInfo.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_ego_vehicle_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaEgoVehicleInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb8, 0xdc, 0x38, 0x66, 0x01, 0x49, 0x24, 0xce,
      0x7f, 0xe5, 0x7e, 0x0c, 0xff, 0x05, 0xa5, 0x44,
      0x8e, 0xc1, 0x8f, 0x27, 0x15, 0x66, 0x56, 0xb3,
      0xb1, 0xb6, 0x9f, 0x4d, 0xa5, 0x58, 0xe9, 0x56,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "carla_msgs/msg/detail/carla_ego_vehicle_info_wheel__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t carla_msgs__msg__CarlaEgoVehicleInfoWheel__EXPECTED_HASH = {1, {
    0x63, 0xca, 0xae, 0x9f, 0x26, 0x37, 0x75, 0x2b,
    0x93, 0xa6, 0x16, 0xb1, 0x0e, 0x00, 0xd2, 0x1b,
    0xe1, 0x5c, 0xa3, 0x2c, 0xac, 0x53, 0x3e, 0xfd,
    0x35, 0x96, 0x34, 0x9a, 0x5c, 0xf4, 0xd4, 0xfe,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
#endif

static char carla_msgs__msg__CarlaEgoVehicleInfo__TYPE_NAME[] = "carla_msgs/msg/CarlaEgoVehicleInfo";
static char carla_msgs__msg__CarlaEgoVehicleInfoWheel__TYPE_NAME[] = "carla_msgs/msg/CarlaEgoVehicleInfoWheel";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__id[] = "id";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__type[] = "type";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__rolename[] = "rolename";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__wheels[] = "wheels";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__max_rpm[] = "max_rpm";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__moi[] = "moi";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__damping_rate_full_throttle[] = "damping_rate_full_throttle";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__damping_rate_zero_throttle_clutch_engaged[] = "damping_rate_zero_throttle_clutch_engaged";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__damping_rate_zero_throttle_clutch_disengaged[] = "damping_rate_zero_throttle_clutch_disengaged";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__use_gear_autobox[] = "use_gear_autobox";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__gear_switch_time[] = "gear_switch_time";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__clutch_strength[] = "clutch_strength";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__mass[] = "mass";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__drag_coefficient[] = "drag_coefficient";
static char carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__center_of_mass[] = "center_of_mass";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaEgoVehicleInfo__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__rolename, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__wheels, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {carla_msgs__msg__CarlaEgoVehicleInfoWheel__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__max_rpm, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__moi, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__damping_rate_full_throttle, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__damping_rate_zero_throttle_clutch_engaged, 41, 41},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__damping_rate_zero_throttle_clutch_disengaged, 44, 44},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__use_gear_autobox, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__gear_switch_time, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__clutch_strength, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__mass, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__drag_coefficient, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaEgoVehicleInfo__FIELD_NAME__center_of_mass, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription carla_msgs__msg__CarlaEgoVehicleInfo__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {carla_msgs__msg__CarlaEgoVehicleInfoWheel__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaEgoVehicleInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaEgoVehicleInfo__TYPE_NAME, 34, 34},
      {carla_msgs__msg__CarlaEgoVehicleInfo__FIELDS, 15, 15},
    },
    {carla_msgs__msg__CarlaEgoVehicleInfo__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&carla_msgs__msg__CarlaEgoVehicleInfoWheel__EXPECTED_HASH, carla_msgs__msg__CarlaEgoVehicleInfoWheel__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = carla_msgs__msg__CarlaEgoVehicleInfoWheel__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
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
  "string type\n"
  "string rolename\n"
  "CarlaEgoVehicleInfoWheel[] wheels\n"
  "float32 max_rpm\n"
  "float32 moi\n"
  "float32 damping_rate_full_throttle\n"
  "float32 damping_rate_zero_throttle_clutch_engaged\n"
  "float32 damping_rate_zero_throttle_clutch_disengaged\n"
  "bool use_gear_autobox\n"
  "float32 gear_switch_time\n"
  "float32 clutch_strength\n"
  "float32 mass\n"
  "float32 drag_coefficient\n"
  "geometry_msgs/Vector3 center_of_mass";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaEgoVehicleInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaEgoVehicleInfo__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 548, 548},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaEgoVehicleInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaEgoVehicleInfo__get_individual_type_description_source(NULL),
    sources[1] = *carla_msgs__msg__CarlaEgoVehicleInfoWheel__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
