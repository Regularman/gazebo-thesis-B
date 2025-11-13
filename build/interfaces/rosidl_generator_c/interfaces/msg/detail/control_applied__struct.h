// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/ControlApplied.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CONTROL_APPLIED__STRUCT_H_
#define INTERFACES__MSG__DETAIL__CONTROL_APPLIED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/ControlApplied in the package interfaces.
typedef struct interfaces__msg__ControlApplied
{
  builtin_interfaces__msg__Time stamp;
  float pose[13];
  float u[4];
  float u_rate[4];
  float est_params[6];
} interfaces__msg__ControlApplied;

// Struct for a sequence of interfaces__msg__ControlApplied.
typedef struct interfaces__msg__ControlApplied__Sequence
{
  interfaces__msg__ControlApplied * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__ControlApplied__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__CONTROL_APPLIED__STRUCT_H_
