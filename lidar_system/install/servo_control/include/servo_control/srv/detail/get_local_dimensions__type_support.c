// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from servo_control:srv/GetLocalDimensions.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "servo_control/srv/detail/get_local_dimensions__rosidl_typesupport_introspection_c.h"
#include "servo_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "servo_control/srv/detail/get_local_dimensions__functions.h"
#include "servo_control/srv/detail/get_local_dimensions__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  servo_control__srv__GetLocalDimensions_Request__init(message_memory);
}

void GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_fini_function(void * message_memory)
{
  servo_control__srv__GetLocalDimensions_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Request, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Request, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Request, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_members = {
  "servo_control__srv",  // message namespace
  "GetLocalDimensions_Request",  // message name
  3,  // number of fields
  sizeof(servo_control__srv__GetLocalDimensions_Request),
  GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_member_array,  // message members
  GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_type_support_handle = {
  0,
  &GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_servo_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_control, srv, GetLocalDimensions_Request)() {
  if (!GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_type_support_handle.typesupport_identifier) {
    GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetLocalDimensions_Request__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "servo_control/srv/detail/get_local_dimensions__rosidl_typesupport_introspection_c.h"
// already included above
// #include "servo_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "servo_control/srv/detail/get_local_dimensions__functions.h"
// already included above
// #include "servo_control/srv/detail/get_local_dimensions__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  servo_control__srv__GetLocalDimensions_Response__init(message_memory);
}

void GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_fini_function(void * message_memory)
{
  servo_control__srv__GetLocalDimensions_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_member_array[7] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "floor_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Response, floor_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ceiling_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Response, ceiling_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "room_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Response, room_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "clearance_below",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Response, clearance_below),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "clearance_above",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_control__srv__GetLocalDimensions_Response, clearance_above),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_members = {
  "servo_control__srv",  // message namespace
  "GetLocalDimensions_Response",  // message name
  7,  // number of fields
  sizeof(servo_control__srv__GetLocalDimensions_Response),
  GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_member_array,  // message members
  GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_type_support_handle = {
  0,
  &GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_servo_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_control, srv, GetLocalDimensions_Response)() {
  if (!GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_type_support_handle.typesupport_identifier) {
    GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetLocalDimensions_Response__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "servo_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "servo_control/srv/detail/get_local_dimensions__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_service_members = {
  "servo_control__srv",  // service namespace
  "GetLocalDimensions",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_Request_message_type_support_handle,
  NULL  // response message
  // servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_Response_message_type_support_handle
};

static rosidl_service_type_support_t servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_service_type_support_handle = {
  0,
  &servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_control, srv, GetLocalDimensions_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_control, srv, GetLocalDimensions_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_servo_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_control, srv, GetLocalDimensions)() {
  if (!servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_service_type_support_handle.typesupport_identifier) {
    servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_control, srv, GetLocalDimensions_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_control, srv, GetLocalDimensions_Response)()->data;
  }

  return &servo_control__srv__detail__get_local_dimensions__rosidl_typesupport_introspection_c__GetLocalDimensions_service_type_support_handle;
}
