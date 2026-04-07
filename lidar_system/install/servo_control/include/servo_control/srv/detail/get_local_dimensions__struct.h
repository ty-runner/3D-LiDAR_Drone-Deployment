// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from servo_control:srv/GetLocalDimensions.idl
// generated code does not contain a copyright notice

#ifndef SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__STRUCT_H_
#define SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/GetLocalDimensions in the package servo_control.
typedef struct servo_control__srv__GetLocalDimensions_Request
{
  double x;
  double y;
  double z;
} servo_control__srv__GetLocalDimensions_Request;

// Struct for a sequence of servo_control__srv__GetLocalDimensions_Request.
typedef struct servo_control__srv__GetLocalDimensions_Request__Sequence
{
  servo_control__srv__GetLocalDimensions_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} servo_control__srv__GetLocalDimensions_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/GetLocalDimensions in the package servo_control.
typedef struct servo_control__srv__GetLocalDimensions_Response
{
  bool success;
  rosidl_runtime_c__String message;
  double floor_z;
  double ceiling_z;
  double room_height;
  double clearance_below;
  double clearance_above;
} servo_control__srv__GetLocalDimensions_Response;

// Struct for a sequence of servo_control__srv__GetLocalDimensions_Response.
typedef struct servo_control__srv__GetLocalDimensions_Response__Sequence
{
  servo_control__srv__GetLocalDimensions_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} servo_control__srv__GetLocalDimensions_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__STRUCT_H_
