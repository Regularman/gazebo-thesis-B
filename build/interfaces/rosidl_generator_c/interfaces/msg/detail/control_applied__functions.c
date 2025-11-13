// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/ControlApplied.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/control_applied__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
interfaces__msg__ControlApplied__init(interfaces__msg__ControlApplied * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    interfaces__msg__ControlApplied__fini(msg);
    return false;
  }
  // pose
  // u
  // u_rate
  // est_params
  return true;
}

void
interfaces__msg__ControlApplied__fini(interfaces__msg__ControlApplied * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // pose
  // u
  // u_rate
  // est_params
}

bool
interfaces__msg__ControlApplied__are_equal(const interfaces__msg__ControlApplied * lhs, const interfaces__msg__ControlApplied * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // pose
  for (size_t i = 0; i < 13; ++i) {
    if (lhs->pose[i] != rhs->pose[i]) {
      return false;
    }
  }
  // u
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->u[i] != rhs->u[i]) {
      return false;
    }
  }
  // u_rate
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->u_rate[i] != rhs->u_rate[i]) {
      return false;
    }
  }
  // est_params
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->est_params[i] != rhs->est_params[i]) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__ControlApplied__copy(
  const interfaces__msg__ControlApplied * input,
  interfaces__msg__ControlApplied * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // pose
  for (size_t i = 0; i < 13; ++i) {
    output->pose[i] = input->pose[i];
  }
  // u
  for (size_t i = 0; i < 4; ++i) {
    output->u[i] = input->u[i];
  }
  // u_rate
  for (size_t i = 0; i < 4; ++i) {
    output->u_rate[i] = input->u_rate[i];
  }
  // est_params
  for (size_t i = 0; i < 6; ++i) {
    output->est_params[i] = input->est_params[i];
  }
  return true;
}

interfaces__msg__ControlApplied *
interfaces__msg__ControlApplied__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__ControlApplied * msg = (interfaces__msg__ControlApplied *)allocator.allocate(sizeof(interfaces__msg__ControlApplied), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__ControlApplied));
  bool success = interfaces__msg__ControlApplied__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__ControlApplied__destroy(interfaces__msg__ControlApplied * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__ControlApplied__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__ControlApplied__Sequence__init(interfaces__msg__ControlApplied__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__ControlApplied * data = NULL;

  if (size) {
    data = (interfaces__msg__ControlApplied *)allocator.zero_allocate(size, sizeof(interfaces__msg__ControlApplied), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__ControlApplied__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__ControlApplied__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interfaces__msg__ControlApplied__Sequence__fini(interfaces__msg__ControlApplied__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interfaces__msg__ControlApplied__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interfaces__msg__ControlApplied__Sequence *
interfaces__msg__ControlApplied__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__ControlApplied__Sequence * array = (interfaces__msg__ControlApplied__Sequence *)allocator.allocate(sizeof(interfaces__msg__ControlApplied__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__ControlApplied__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__ControlApplied__Sequence__destroy(interfaces__msg__ControlApplied__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__ControlApplied__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__ControlApplied__Sequence__are_equal(const interfaces__msg__ControlApplied__Sequence * lhs, const interfaces__msg__ControlApplied__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__ControlApplied__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__ControlApplied__Sequence__copy(
  const interfaces__msg__ControlApplied__Sequence * input,
  interfaces__msg__ControlApplied__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__ControlApplied);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__ControlApplied * data =
      (interfaces__msg__ControlApplied *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__ControlApplied__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__ControlApplied__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__ControlApplied__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
