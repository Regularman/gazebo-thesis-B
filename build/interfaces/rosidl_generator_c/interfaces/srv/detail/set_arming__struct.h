// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/SetArming.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__SET_ARMING__STRUCT_H_
#define INTERFACES__SRV__DETAIL__SET_ARMING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetArming in the package interfaces.
typedef struct interfaces__srv__SetArming_Request
{
  /// true to arm, false to disarm
  bool arm;
} interfaces__srv__SetArming_Request;

// Struct for a sequence of interfaces__srv__SetArming_Request.
typedef struct interfaces__srv__SetArming_Request__Sequence
{
  interfaces__srv__SetArming_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__SetArming_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetArming in the package interfaces.
typedef struct interfaces__srv__SetArming_Response
{
  /// true if the command was accepted
  bool success;
  /// optional status message
  rosidl_runtime_c__String message;
} interfaces__srv__SetArming_Response;

// Struct for a sequence of interfaces__srv__SetArming_Response.
typedef struct interfaces__srv__SetArming_Response__Sequence
{
  interfaces__srv__SetArming_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__SetArming_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__SET_ARMING__STRUCT_H_
