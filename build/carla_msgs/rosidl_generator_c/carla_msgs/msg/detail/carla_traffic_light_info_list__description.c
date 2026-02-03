// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaTrafficLightInfoList.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_traffic_light_info_list__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaTrafficLightInfoList__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0x13, 0xec, 0xb7, 0xab, 0xe8, 0xb5, 0xb5,
      0x3c, 0x9d, 0xc5, 0xb7, 0x79, 0x49, 0x81, 0xc9,
      0x0b, 0xd0, 0x4b, 0x05, 0x56, 0xf8, 0xab, 0x66,
      0xcf, 0x26, 0x86, 0x3a, 0xc6, 0x00, 0xfd, 0xe6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "carla_msgs/msg/detail/carla_traffic_light_info__functions.h"
#include "carla_msgs/msg/detail/carla_bounding_box__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t carla_msgs__msg__CarlaBoundingBox__EXPECTED_HASH = {1, {
    0x64, 0x15, 0x0a, 0x58, 0xba, 0xdd, 0xf4, 0xec,
    0xdf, 0xd2, 0xad, 0xcc, 0x41, 0xb7, 0x4b, 0xc2,
    0xa7, 0xca, 0xaa, 0x16, 0x77, 0x54, 0x5e, 0x97,
    0x1b, 0x47, 0x62, 0x3c, 0xa1, 0xc3, 0x1b, 0x63,
  }};
static const rosidl_type_hash_t carla_msgs__msg__CarlaTrafficLightInfo__EXPECTED_HASH = {1, {
    0xc6, 0xbc, 0x35, 0x8a, 0x73, 0x6c, 0x08, 0x43,
    0x88, 0x9a, 0x6f, 0x65, 0xd8, 0x07, 0x0f, 0x98,
    0x91, 0x61, 0x9a, 0x25, 0x0f, 0xcc, 0x58, 0xad,
    0x51, 0xff, 0xb9, 0xf8, 0x7d, 0xd1, 0x38, 0xb4,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
#endif

static char carla_msgs__msg__CarlaTrafficLightInfoList__TYPE_NAME[] = "carla_msgs/msg/CarlaTrafficLightInfoList";
static char carla_msgs__msg__CarlaBoundingBox__TYPE_NAME[] = "carla_msgs/msg/CarlaBoundingBox";
static char carla_msgs__msg__CarlaTrafficLightInfo__TYPE_NAME[] = "carla_msgs/msg/CarlaTrafficLightInfo";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaTrafficLightInfoList__FIELD_NAME__traffic_lights[] = "traffic_lights";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaTrafficLightInfoList__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaTrafficLightInfoList__FIELD_NAME__traffic_lights, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {carla_msgs__msg__CarlaTrafficLightInfo__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription carla_msgs__msg__CarlaTrafficLightInfoList__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {carla_msgs__msg__CarlaBoundingBox__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaTrafficLightInfo__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaTrafficLightInfoList__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaTrafficLightInfoList__TYPE_NAME, 40, 40},
      {carla_msgs__msg__CarlaTrafficLightInfoList__FIELDS, 1, 1},
    },
    {carla_msgs__msg__CarlaTrafficLightInfoList__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&carla_msgs__msg__CarlaBoundingBox__EXPECTED_HASH, carla_msgs__msg__CarlaBoundingBox__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = carla_msgs__msg__CarlaBoundingBox__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&carla_msgs__msg__CarlaTrafficLightInfo__EXPECTED_HASH, carla_msgs__msg__CarlaTrafficLightInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = carla_msgs__msg__CarlaTrafficLightInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
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
  "CarlaTrafficLightInfo[] traffic_lights";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaTrafficLightInfoList__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaTrafficLightInfoList__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 202, 202},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaTrafficLightInfoList__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaTrafficLightInfoList__get_individual_type_description_source(NULL),
    sources[1] = *carla_msgs__msg__CarlaBoundingBox__get_individual_type_description_source(NULL);
    sources[2] = *carla_msgs__msg__CarlaTrafficLightInfo__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
