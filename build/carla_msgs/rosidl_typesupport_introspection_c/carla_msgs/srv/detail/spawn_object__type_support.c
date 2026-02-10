// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from carla_msgs:srv/SpawnObject.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "carla_msgs/srv/detail/spawn_object__rosidl_typesupport_introspection_c.h"
#include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "carla_msgs/srv/detail/spawn_object__functions.h"
#include "carla_msgs/srv/detail/spawn_object__struct.h"


// Include directives for member types
// Member `type`
// Member `id`
#include "rosidl_runtime_c/string_functions.h"
// Member `attributes`
#include "diagnostic_msgs/msg/key_value.h"
// Member `attributes`
#include "diagnostic_msgs/msg/detail/key_value__rosidl_typesupport_introspection_c.h"
// Member `transform`
#include "geometry_msgs/msg/pose.h"
// Member `transform`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_msgs__srv__SpawnObject_Request__init(message_memory);
}

void carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_fini_function(void * message_memory)
{
  carla_msgs__srv__SpawnObject_Request__fini(message_memory);
}

size_t carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__size_function__SpawnObject_Request__attributes(
  const void * untyped_member)
{
  const diagnostic_msgs__msg__KeyValue__Sequence * member =
    (const diagnostic_msgs__msg__KeyValue__Sequence *)(untyped_member);
  return member->size;
}

const void * carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Request__attributes(
  const void * untyped_member, size_t index)
{
  const diagnostic_msgs__msg__KeyValue__Sequence * member =
    (const diagnostic_msgs__msg__KeyValue__Sequence *)(untyped_member);
  return &member->data[index];
}

void * carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__get_function__SpawnObject_Request__attributes(
  void * untyped_member, size_t index)
{
  diagnostic_msgs__msg__KeyValue__Sequence * member =
    (diagnostic_msgs__msg__KeyValue__Sequence *)(untyped_member);
  return &member->data[index];
}

void carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__fetch_function__SpawnObject_Request__attributes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const diagnostic_msgs__msg__KeyValue * item =
    ((const diagnostic_msgs__msg__KeyValue *)
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Request__attributes(untyped_member, index));
  diagnostic_msgs__msg__KeyValue * value =
    (diagnostic_msgs__msg__KeyValue *)(untyped_value);
  *value = *item;
}

void carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__assign_function__SpawnObject_Request__attributes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  diagnostic_msgs__msg__KeyValue * item =
    ((diagnostic_msgs__msg__KeyValue *)
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__get_function__SpawnObject_Request__attributes(untyped_member, index));
  const diagnostic_msgs__msg__KeyValue * value =
    (const diagnostic_msgs__msg__KeyValue *)(untyped_value);
  *item = *value;
}

bool carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__resize_function__SpawnObject_Request__attributes(
  void * untyped_member, size_t size)
{
  diagnostic_msgs__msg__KeyValue__Sequence * member =
    (diagnostic_msgs__msg__KeyValue__Sequence *)(untyped_member);
  diagnostic_msgs__msg__KeyValue__Sequence__fini(member);
  return diagnostic_msgs__msg__KeyValue__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_member_array[6] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Request, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Request, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "attributes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Request, attributes),  // bytes offset in struct
    NULL,  // default value
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__size_function__SpawnObject_Request__attributes,  // size() function pointer
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Request__attributes,  // get_const(index) function pointer
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__get_function__SpawnObject_Request__attributes,  // get(index) function pointer
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__fetch_function__SpawnObject_Request__attributes,  // fetch(index, &value) function pointer
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__assign_function__SpawnObject_Request__attributes,  // assign(index, value) function pointer
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__resize_function__SpawnObject_Request__attributes  // resize(index) function pointer
  },
  {
    "transform",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Request, transform),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "attach_to",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Request, attach_to),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "random_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Request, random_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_members = {
  "carla_msgs__srv",  // message namespace
  "SpawnObject_Request",  // message name
  6,  // number of fields
  sizeof(carla_msgs__srv__SpawnObject_Request),
  false,  // has_any_key_member_
  carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_member_array,  // message members
  carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_type_support_handle = {
  0,
  &carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_members,
  get_message_typesupport_handle_function,
  &carla_msgs__srv__SpawnObject_Request__get_type_hash,
  &carla_msgs__srv__SpawnObject_Request__get_type_description,
  &carla_msgs__srv__SpawnObject_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Request)() {
  carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, diagnostic_msgs, msg, KeyValue)();
  carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_type_support_handle.typesupport_identifier) {
    carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "carla_msgs/srv/detail/spawn_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "carla_msgs/srv/detail/spawn_object__functions.h"
// already included above
// #include "carla_msgs/srv/detail/spawn_object__struct.h"


// Include directives for member types
// Member `error_string`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_msgs__srv__SpawnObject_Response__init(message_memory);
}

void carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_fini_function(void * message_memory)
{
  carla_msgs__srv__SpawnObject_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_member_array[2] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Response, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_string",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Response, error_string),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_members = {
  "carla_msgs__srv",  // message namespace
  "SpawnObject_Response",  // message name
  2,  // number of fields
  sizeof(carla_msgs__srv__SpawnObject_Response),
  false,  // has_any_key_member_
  carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_member_array,  // message members
  carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_type_support_handle = {
  0,
  &carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_members,
  get_message_typesupport_handle_function,
  &carla_msgs__srv__SpawnObject_Response__get_type_hash,
  &carla_msgs__srv__SpawnObject_Response__get_type_description,
  &carla_msgs__srv__SpawnObject_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Response)() {
  if (!carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_type_support_handle.typesupport_identifier) {
    carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "carla_msgs/srv/detail/spawn_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "carla_msgs/srv/detail/spawn_object__functions.h"
// already included above
// #include "carla_msgs/srv/detail/spawn_object__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "carla_msgs/srv/spawn_object.h"
// Member `request`
// Member `response`
// already included above
// #include "carla_msgs/srv/detail/spawn_object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_msgs__srv__SpawnObject_Event__init(message_memory);
}

void carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_fini_function(void * message_memory)
{
  carla_msgs__srv__SpawnObject_Event__fini(message_memory);
}

size_t carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__size_function__SpawnObject_Event__request(
  const void * untyped_member)
{
  const carla_msgs__srv__SpawnObject_Request__Sequence * member =
    (const carla_msgs__srv__SpawnObject_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Event__request(
  const void * untyped_member, size_t index)
{
  const carla_msgs__srv__SpawnObject_Request__Sequence * member =
    (const carla_msgs__srv__SpawnObject_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_function__SpawnObject_Event__request(
  void * untyped_member, size_t index)
{
  carla_msgs__srv__SpawnObject_Request__Sequence * member =
    (carla_msgs__srv__SpawnObject_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__fetch_function__SpawnObject_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const carla_msgs__srv__SpawnObject_Request * item =
    ((const carla_msgs__srv__SpawnObject_Request *)
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Event__request(untyped_member, index));
  carla_msgs__srv__SpawnObject_Request * value =
    (carla_msgs__srv__SpawnObject_Request *)(untyped_value);
  *value = *item;
}

void carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__assign_function__SpawnObject_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  carla_msgs__srv__SpawnObject_Request * item =
    ((carla_msgs__srv__SpawnObject_Request *)
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_function__SpawnObject_Event__request(untyped_member, index));
  const carla_msgs__srv__SpawnObject_Request * value =
    (const carla_msgs__srv__SpawnObject_Request *)(untyped_value);
  *item = *value;
}

bool carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__resize_function__SpawnObject_Event__request(
  void * untyped_member, size_t size)
{
  carla_msgs__srv__SpawnObject_Request__Sequence * member =
    (carla_msgs__srv__SpawnObject_Request__Sequence *)(untyped_member);
  carla_msgs__srv__SpawnObject_Request__Sequence__fini(member);
  return carla_msgs__srv__SpawnObject_Request__Sequence__init(member, size);
}

size_t carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__size_function__SpawnObject_Event__response(
  const void * untyped_member)
{
  const carla_msgs__srv__SpawnObject_Response__Sequence * member =
    (const carla_msgs__srv__SpawnObject_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Event__response(
  const void * untyped_member, size_t index)
{
  const carla_msgs__srv__SpawnObject_Response__Sequence * member =
    (const carla_msgs__srv__SpawnObject_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_function__SpawnObject_Event__response(
  void * untyped_member, size_t index)
{
  carla_msgs__srv__SpawnObject_Response__Sequence * member =
    (carla_msgs__srv__SpawnObject_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__fetch_function__SpawnObject_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const carla_msgs__srv__SpawnObject_Response * item =
    ((const carla_msgs__srv__SpawnObject_Response *)
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Event__response(untyped_member, index));
  carla_msgs__srv__SpawnObject_Response * value =
    (carla_msgs__srv__SpawnObject_Response *)(untyped_value);
  *value = *item;
}

void carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__assign_function__SpawnObject_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  carla_msgs__srv__SpawnObject_Response * item =
    ((carla_msgs__srv__SpawnObject_Response *)
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_function__SpawnObject_Event__response(untyped_member, index));
  const carla_msgs__srv__SpawnObject_Response * value =
    (const carla_msgs__srv__SpawnObject_Response *)(untyped_value);
  *item = *value;
}

bool carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__resize_function__SpawnObject_Event__response(
  void * untyped_member, size_t size)
{
  carla_msgs__srv__SpawnObject_Response__Sequence * member =
    (carla_msgs__srv__SpawnObject_Response__Sequence *)(untyped_member);
  carla_msgs__srv__SpawnObject_Response__Sequence__fini(member);
  return carla_msgs__srv__SpawnObject_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Event, request),  // bytes offset in struct
    NULL,  // default value
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__size_function__SpawnObject_Event__request,  // size() function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Event__request,  // get_const(index) function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_function__SpawnObject_Event__request,  // get(index) function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__fetch_function__SpawnObject_Event__request,  // fetch(index, &value) function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__assign_function__SpawnObject_Event__request,  // assign(index, value) function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__resize_function__SpawnObject_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(carla_msgs__srv__SpawnObject_Event, response),  // bytes offset in struct
    NULL,  // default value
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__size_function__SpawnObject_Event__response,  // size() function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_const_function__SpawnObject_Event__response,  // get_const(index) function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__get_function__SpawnObject_Event__response,  // get(index) function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__fetch_function__SpawnObject_Event__response,  // fetch(index, &value) function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__assign_function__SpawnObject_Event__response,  // assign(index, value) function pointer
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__resize_function__SpawnObject_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_members = {
  "carla_msgs__srv",  // message namespace
  "SpawnObject_Event",  // message name
  3,  // number of fields
  sizeof(carla_msgs__srv__SpawnObject_Event),
  false,  // has_any_key_member_
  carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_member_array,  // message members
  carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_type_support_handle = {
  0,
  &carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_members,
  get_message_typesupport_handle_function,
  &carla_msgs__srv__SpawnObject_Event__get_type_hash,
  &carla_msgs__srv__SpawnObject_Event__get_type_description,
  &carla_msgs__srv__SpawnObject_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Event)() {
  carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Request)();
  carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Response)();
  if (!carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_type_support_handle.typesupport_identifier) {
    carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "carla_msgs/srv/detail/spawn_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_service_members = {
  "carla_msgs__srv",  // service namespace
  "SpawnObject",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_Request_message_type_support_handle,
  NULL,  // response message
  // carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_Response_message_type_support_handle
  NULL  // event_message
  // carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_Response_message_type_support_handle
};


static rosidl_service_type_support_t carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_service_type_support_handle = {
  0,
  &carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_service_members,
  get_service_typesupport_handle_function,
  &carla_msgs__srv__SpawnObject_Request__rosidl_typesupport_introspection_c__SpawnObject_Request_message_type_support_handle,
  &carla_msgs__srv__SpawnObject_Response__rosidl_typesupport_introspection_c__SpawnObject_Response_message_type_support_handle,
  &carla_msgs__srv__SpawnObject_Event__rosidl_typesupport_introspection_c__SpawnObject_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    carla_msgs,
    srv,
    SpawnObject
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    carla_msgs,
    srv,
    SpawnObject
  ),
  &carla_msgs__srv__SpawnObject__get_type_hash,
  &carla_msgs__srv__SpawnObject__get_type_description,
  &carla_msgs__srv__SpawnObject__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject)(void) {
  if (!carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_service_type_support_handle.typesupport_identifier) {
    carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, SpawnObject_Event)()->data;
  }

  return &carla_msgs__srv__detail__spawn_object__rosidl_typesupport_introspection_c__SpawnObject_service_type_support_handle;
}
