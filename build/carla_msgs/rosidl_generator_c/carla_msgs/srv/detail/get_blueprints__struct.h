// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from carla_msgs:srv/GetBlueprints.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "carla_msgs/srv/get_blueprints.h"


#ifndef CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__STRUCT_H_
#define CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'filter'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetBlueprints in the package carla_msgs.
typedef struct carla_msgs__srv__GetBlueprints_Request
{
  rosidl_runtime_c__String filter;
} carla_msgs__srv__GetBlueprints_Request;

// Struct for a sequence of carla_msgs__srv__GetBlueprints_Request.
typedef struct carla_msgs__srv__GetBlueprints_Request__Sequence
{
  carla_msgs__srv__GetBlueprints_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} carla_msgs__srv__GetBlueprints_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'blueprints'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetBlueprints in the package carla_msgs.
typedef struct carla_msgs__srv__GetBlueprints_Response
{
  rosidl_runtime_c__String__Sequence blueprints;
} carla_msgs__srv__GetBlueprints_Response;

// Struct for a sequence of carla_msgs__srv__GetBlueprints_Response.
typedef struct carla_msgs__srv__GetBlueprints_Response__Sequence
{
  carla_msgs__srv__GetBlueprints_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} carla_msgs__srv__GetBlueprints_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  carla_msgs__srv__GetBlueprints_Event__request__MAX_SIZE = 1
};
// response
enum
{
  carla_msgs__srv__GetBlueprints_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetBlueprints in the package carla_msgs.
typedef struct carla_msgs__srv__GetBlueprints_Event
{
  service_msgs__msg__ServiceEventInfo info;
  carla_msgs__srv__GetBlueprints_Request__Sequence request;
  carla_msgs__srv__GetBlueprints_Response__Sequence response;
} carla_msgs__srv__GetBlueprints_Event;

// Struct for a sequence of carla_msgs__srv__GetBlueprints_Event.
typedef struct carla_msgs__srv__GetBlueprints_Event__Sequence
{
  carla_msgs__srv__GetBlueprints_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} carla_msgs__srv__GetBlueprints_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__STRUCT_H_
