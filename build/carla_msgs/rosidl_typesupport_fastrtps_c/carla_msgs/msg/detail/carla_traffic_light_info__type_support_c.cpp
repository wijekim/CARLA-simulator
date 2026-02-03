// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from carla_msgs:msg/CarlaTrafficLightInfo.idl
// generated code does not contain a copyright notice
#include "carla_msgs/msg/detail/carla_traffic_light_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "carla_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "carla_msgs/msg/detail/carla_traffic_light_info__struct.h"
#include "carla_msgs/msg/detail/carla_traffic_light_info__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "carla_msgs/msg/detail/carla_bounding_box__functions.h"  // trigger_volume
#include "geometry_msgs/msg/detail/pose__functions.h"  // transform

// forward declare type support functions

bool cdr_serialize_carla_msgs__msg__CarlaBoundingBox(
  const carla_msgs__msg__CarlaBoundingBox * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_carla_msgs__msg__CarlaBoundingBox(
  eprosima::fastcdr::Cdr & cdr,
  carla_msgs__msg__CarlaBoundingBox * ros_message);

size_t get_serialized_size_carla_msgs__msg__CarlaBoundingBox(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_carla_msgs__msg__CarlaBoundingBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_carla_msgs__msg__CarlaBoundingBox(
  const carla_msgs__msg__CarlaBoundingBox * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_carla_msgs__msg__CarlaBoundingBox(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_carla_msgs__msg__CarlaBoundingBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_msgs, msg, CarlaBoundingBox)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_msgs
bool cdr_serialize_geometry_msgs__msg__Pose(
  const geometry_msgs__msg__Pose * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_msgs
bool cdr_deserialize_geometry_msgs__msg__Pose(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__Pose * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_msgs
size_t get_serialized_size_geometry_msgs__msg__Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_msgs
size_t max_serialized_size_geometry_msgs__msg__Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_msgs
bool cdr_serialize_key_geometry_msgs__msg__Pose(
  const geometry_msgs__msg__Pose * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_msgs
size_t get_serialized_size_key_geometry_msgs__msg__Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_msgs
size_t max_serialized_size_key_geometry_msgs__msg__Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose)();


using _CarlaTrafficLightInfo__ros_msg_type = carla_msgs__msg__CarlaTrafficLightInfo;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
bool cdr_serialize_carla_msgs__msg__CarlaTrafficLightInfo(
  const carla_msgs__msg__CarlaTrafficLightInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: transform
  {
    cdr_serialize_geometry_msgs__msg__Pose(
      &ros_message->transform, cdr);
  }

  // Field name: trigger_volume
  {
    cdr_serialize_carla_msgs__msg__CarlaBoundingBox(
      &ros_message->trigger_volume, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
bool cdr_deserialize_carla_msgs__msg__CarlaTrafficLightInfo(
  eprosima::fastcdr::Cdr & cdr,
  carla_msgs__msg__CarlaTrafficLightInfo * ros_message)
{
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: transform
  {
    cdr_deserialize_geometry_msgs__msg__Pose(cdr, &ros_message->transform);
  }

  // Field name: trigger_volume
  {
    cdr_deserialize_carla_msgs__msg__CarlaBoundingBox(cdr, &ros_message->trigger_volume);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
size_t get_serialized_size_carla_msgs__msg__CarlaTrafficLightInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CarlaTrafficLightInfo__ros_msg_type * ros_message = static_cast<const _CarlaTrafficLightInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: transform
  current_alignment += get_serialized_size_geometry_msgs__msg__Pose(
    &(ros_message->transform), current_alignment);

  // Field name: trigger_volume
  current_alignment += get_serialized_size_carla_msgs__msg__CarlaBoundingBox(
    &(ros_message->trigger_volume), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
size_t max_serialized_size_carla_msgs__msg__CarlaTrafficLightInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: transform
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Pose(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: trigger_volume
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_carla_msgs__msg__CarlaBoundingBox(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = carla_msgs__msg__CarlaTrafficLightInfo;
    is_plain =
      (
      offsetof(DataType, trigger_volume) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
bool cdr_serialize_key_carla_msgs__msg__CarlaTrafficLightInfo(
  const carla_msgs__msg__CarlaTrafficLightInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: transform
  {
    cdr_serialize_key_geometry_msgs__msg__Pose(
      &ros_message->transform, cdr);
  }

  // Field name: trigger_volume
  {
    cdr_serialize_key_carla_msgs__msg__CarlaBoundingBox(
      &ros_message->trigger_volume, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
size_t get_serialized_size_key_carla_msgs__msg__CarlaTrafficLightInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CarlaTrafficLightInfo__ros_msg_type * ros_message = static_cast<const _CarlaTrafficLightInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: transform
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Pose(
    &(ros_message->transform), current_alignment);

  // Field name: trigger_volume
  current_alignment += get_serialized_size_key_carla_msgs__msg__CarlaBoundingBox(
    &(ros_message->trigger_volume), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
size_t max_serialized_size_key_carla_msgs__msg__CarlaTrafficLightInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: transform
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Pose(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: trigger_volume
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_carla_msgs__msg__CarlaBoundingBox(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = carla_msgs__msg__CarlaTrafficLightInfo;
    is_plain =
      (
      offsetof(DataType, trigger_volume) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _CarlaTrafficLightInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const carla_msgs__msg__CarlaTrafficLightInfo * ros_message = static_cast<const carla_msgs__msg__CarlaTrafficLightInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_carla_msgs__msg__CarlaTrafficLightInfo(ros_message, cdr);
}

static bool _CarlaTrafficLightInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  carla_msgs__msg__CarlaTrafficLightInfo * ros_message = static_cast<carla_msgs__msg__CarlaTrafficLightInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_carla_msgs__msg__CarlaTrafficLightInfo(cdr, ros_message);
}

static uint32_t _CarlaTrafficLightInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_carla_msgs__msg__CarlaTrafficLightInfo(
      untyped_ros_message, 0));
}

static size_t _CarlaTrafficLightInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_carla_msgs__msg__CarlaTrafficLightInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CarlaTrafficLightInfo = {
  "carla_msgs::msg",
  "CarlaTrafficLightInfo",
  _CarlaTrafficLightInfo__cdr_serialize,
  _CarlaTrafficLightInfo__cdr_deserialize,
  _CarlaTrafficLightInfo__get_serialized_size,
  _CarlaTrafficLightInfo__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _CarlaTrafficLightInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CarlaTrafficLightInfo,
  get_message_typesupport_handle_function,
  &carla_msgs__msg__CarlaTrafficLightInfo__get_type_hash,
  &carla_msgs__msg__CarlaTrafficLightInfo__get_type_description,
  &carla_msgs__msg__CarlaTrafficLightInfo__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_msgs, msg, CarlaTrafficLightInfo)() {
  return &_CarlaTrafficLightInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
