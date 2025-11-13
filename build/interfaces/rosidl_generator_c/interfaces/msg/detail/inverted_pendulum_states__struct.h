// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/InvertedPendulumStates.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__INVERTED_PENDULUM_STATES__STRUCT_H_
#define INTERFACES__MSG__DETAIL__INVERTED_PENDULUM_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'child_frame_id'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/InvertedPendulumStates in the package interfaces.
typedef struct interfaces__msg__InvertedPendulumStates
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String child_frame_id;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist twist;
} interfaces__msg__InvertedPendulumStates;

// Struct for a sequence of interfaces__msg__InvertedPendulumStates.
typedef struct interfaces__msg__InvertedPendulumStates__Sequence
{
  interfaces__msg__InvertedPendulumStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__InvertedPendulumStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__INVERTED_PENDULUM_STATES__STRUCT_H_
