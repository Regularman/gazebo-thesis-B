// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interfaces:msg/ControlApplied.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interfaces/msg/detail/control_applied__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ControlApplied_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interfaces::msg::ControlApplied(_init);
}

void ControlApplied_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interfaces::msg::ControlApplied *>(message_memory);
  typed_message->~ControlApplied();
}

size_t size_function__ControlApplied__pose(const void * untyped_member)
{
  (void)untyped_member;
  return 13;
}

const void * get_const_function__ControlApplied__pose(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 13> *>(untyped_member);
  return &member[index];
}

void * get_function__ControlApplied__pose(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 13> *>(untyped_member);
  return &member[index];
}

void fetch_function__ControlApplied__pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ControlApplied__pose(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ControlApplied__pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ControlApplied__pose(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ControlApplied__u(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ControlApplied__u(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ControlApplied__u(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ControlApplied__u(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ControlApplied__u(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ControlApplied__u(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ControlApplied__u(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ControlApplied__u_rate(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ControlApplied__u_rate(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ControlApplied__u_rate(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ControlApplied__u_rate(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ControlApplied__u_rate(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ControlApplied__u_rate(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ControlApplied__u_rate(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ControlApplied__est_params(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__ControlApplied__est_params(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__ControlApplied__est_params(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__ControlApplied__est_params(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ControlApplied__est_params(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ControlApplied__est_params(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ControlApplied__est_params(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ControlApplied_message_member_array[5] = {
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::ControlApplied, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    13,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::ControlApplied, pose),  // bytes offset in struct
    nullptr,  // default value
    size_function__ControlApplied__pose,  // size() function pointer
    get_const_function__ControlApplied__pose,  // get_const(index) function pointer
    get_function__ControlApplied__pose,  // get(index) function pointer
    fetch_function__ControlApplied__pose,  // fetch(index, &value) function pointer
    assign_function__ControlApplied__pose,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "u",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::ControlApplied, u),  // bytes offset in struct
    nullptr,  // default value
    size_function__ControlApplied__u,  // size() function pointer
    get_const_function__ControlApplied__u,  // get_const(index) function pointer
    get_function__ControlApplied__u,  // get(index) function pointer
    fetch_function__ControlApplied__u,  // fetch(index, &value) function pointer
    assign_function__ControlApplied__u,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "u_rate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::ControlApplied, u_rate),  // bytes offset in struct
    nullptr,  // default value
    size_function__ControlApplied__u_rate,  // size() function pointer
    get_const_function__ControlApplied__u_rate,  // get_const(index) function pointer
    get_function__ControlApplied__u_rate,  // get(index) function pointer
    fetch_function__ControlApplied__u_rate,  // fetch(index, &value) function pointer
    assign_function__ControlApplied__u_rate,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "est_params",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::ControlApplied, est_params),  // bytes offset in struct
    nullptr,  // default value
    size_function__ControlApplied__est_params,  // size() function pointer
    get_const_function__ControlApplied__est_params,  // get_const(index) function pointer
    get_function__ControlApplied__est_params,  // get(index) function pointer
    fetch_function__ControlApplied__est_params,  // fetch(index, &value) function pointer
    assign_function__ControlApplied__est_params,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ControlApplied_message_members = {
  "interfaces::msg",  // message namespace
  "ControlApplied",  // message name
  5,  // number of fields
  sizeof(interfaces::msg::ControlApplied),
  ControlApplied_message_member_array,  // message members
  ControlApplied_init_function,  // function to initialize message memory (memory has to be allocated)
  ControlApplied_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ControlApplied_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ControlApplied_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<interfaces::msg::ControlApplied>()
{
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::ControlApplied_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interfaces, msg, ControlApplied)() {
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::ControlApplied_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
