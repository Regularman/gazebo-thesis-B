// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/ControlApplied.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CONTROL_APPLIED__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__CONTROL_APPLIED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interfaces__msg__ControlApplied __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__ControlApplied __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControlApplied_
{
  using Type = ControlApplied_<ContainerAllocator>;

  explicit ControlApplied_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 13>::iterator, float>(this->pose.begin(), this->pose.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->u.begin(), this->u.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->u_rate.begin(), this->u_rate.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->est_params.begin(), this->est_params.end(), 0.0f);
    }
  }

  explicit ControlApplied_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    pose(_alloc),
    u(_alloc),
    u_rate(_alloc),
    est_params(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 13>::iterator, float>(this->pose.begin(), this->pose.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->u.begin(), this->u.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->u_rate.begin(), this->u_rate.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->est_params.begin(), this->est_params.end(), 0.0f);
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _pose_type =
    std::array<float, 13>;
  _pose_type pose;
  using _u_type =
    std::array<float, 4>;
  _u_type u;
  using _u_rate_type =
    std::array<float, 4>;
  _u_rate_type u_rate;
  using _est_params_type =
    std::array<float, 6>;
  _est_params_type est_params;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__pose(
    const std::array<float, 13> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__u(
    const std::array<float, 4> & _arg)
  {
    this->u = _arg;
    return *this;
  }
  Type & set__u_rate(
    const std::array<float, 4> & _arg)
  {
    this->u_rate = _arg;
    return *this;
  }
  Type & set__est_params(
    const std::array<float, 6> & _arg)
  {
    this->est_params = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::ControlApplied_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::ControlApplied_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::ControlApplied_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::ControlApplied_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::ControlApplied_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::ControlApplied_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::ControlApplied_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::ControlApplied_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::ControlApplied_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::ControlApplied_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__ControlApplied
    std::shared_ptr<interfaces::msg::ControlApplied_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__ControlApplied
    std::shared_ptr<interfaces::msg::ControlApplied_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlApplied_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->u != other.u) {
      return false;
    }
    if (this->u_rate != other.u_rate) {
      return false;
    }
    if (this->est_params != other.est_params) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlApplied_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlApplied_

// alias to use template instance with default allocator
using ControlApplied =
  interfaces::msg::ControlApplied_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CONTROL_APPLIED__STRUCT_HPP_
