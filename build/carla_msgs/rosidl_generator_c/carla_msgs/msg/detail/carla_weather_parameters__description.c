// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from carla_msgs:msg/CarlaWeatherParameters.idl
// generated code does not contain a copyright notice

#include "carla_msgs/msg/detail/carla_weather_parameters__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_carla_msgs
const rosidl_type_hash_t *
carla_msgs__msg__CarlaWeatherParameters__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x85, 0xe1, 0xbb, 0xa9, 0xe8, 0xfc, 0x0f, 0x7b,
      0xf7, 0x69, 0xd0, 0x97, 0x71, 0xc4, 0xb2, 0xb7,
      0x5f, 0xf9, 0x70, 0x2c, 0xd4, 0x86, 0xdf, 0x24,
      0xe1, 0x53, 0xc7, 0x42, 0x32, 0xe2, 0x62, 0x16,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char carla_msgs__msg__CarlaWeatherParameters__TYPE_NAME[] = "carla_msgs/msg/CarlaWeatherParameters";

// Define type names, field names, and default values
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__cloudiness[] = "cloudiness";
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__precipitation[] = "precipitation";
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__precipitation_deposits[] = "precipitation_deposits";
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__wind_intensity[] = "wind_intensity";
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__fog_density[] = "fog_density";
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__fog_distance[] = "fog_distance";
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__wetness[] = "wetness";
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__sun_azimuth_angle[] = "sun_azimuth_angle";
static char carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__sun_altitude_angle[] = "sun_altitude_angle";

static rosidl_runtime_c__type_description__Field carla_msgs__msg__CarlaWeatherParameters__FIELDS[] = {
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__cloudiness, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__precipitation, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__precipitation_deposits, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__wind_intensity, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__fog_density, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__fog_distance, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__wetness, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__sun_azimuth_angle, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {carla_msgs__msg__CarlaWeatherParameters__FIELD_NAME__sun_altitude_angle, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
carla_msgs__msg__CarlaWeatherParameters__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {carla_msgs__msg__CarlaWeatherParameters__TYPE_NAME, 37, 37},
      {carla_msgs__msg__CarlaWeatherParameters__FIELDS, 9, 9},
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
  "\n"
  "float32 cloudiness\n"
  "float32 precipitation\n"
  "float32 precipitation_deposits\n"
  "float32 wind_intensity\n"
  "float32 fog_density\n"
  "float32 fog_distance\n"
  "float32 wetness\n"
  "float32 sun_azimuth_angle\n"
  "float32 sun_altitude_angle";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
carla_msgs__msg__CarlaWeatherParameters__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {carla_msgs__msg__CarlaWeatherParameters__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 369, 369},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
carla_msgs__msg__CarlaWeatherParameters__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *carla_msgs__msg__CarlaWeatherParameters__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
