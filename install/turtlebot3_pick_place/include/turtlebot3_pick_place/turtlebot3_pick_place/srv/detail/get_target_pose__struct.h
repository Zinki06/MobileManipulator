// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_pick_place:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__STRUCT_H_
#define TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_class'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetTargetPose in the package turtlebot3_pick_place.
typedef struct turtlebot3_pick_place__srv__GetTargetPose_Request
{
  rosidl_runtime_c__String target_class;
} turtlebot3_pick_place__srv__GetTargetPose_Request;

// Struct for a sequence of turtlebot3_pick_place__srv__GetTargetPose_Request.
typedef struct turtlebot3_pick_place__srv__GetTargetPose_Request__Sequence
{
  turtlebot3_pick_place__srv__GetTargetPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_pick_place__srv__GetTargetPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/GetTargetPose in the package turtlebot3_pick_place.
typedef struct turtlebot3_pick_place__srv__GetTargetPose_Response
{
  bool success;
  rosidl_runtime_c__String message;
  geometry_msgs__msg__PoseStamped target_pose;
} turtlebot3_pick_place__srv__GetTargetPose_Response;

// Struct for a sequence of turtlebot3_pick_place__srv__GetTargetPose_Response.
typedef struct turtlebot3_pick_place__srv__GetTargetPose_Response__Sequence
{
  turtlebot3_pick_place__srv__GetTargetPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_pick_place__srv__GetTargetPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_PICK_PLACE__SRV__DETAIL__GET_TARGET_POSE__STRUCT_H_
