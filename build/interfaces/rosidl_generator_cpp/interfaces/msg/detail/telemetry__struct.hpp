// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Telemetry.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__TELEMETRY__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__TELEMETRY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Telemetry __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Telemetry __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Telemetry_
{
  using Type = Telemetry_<ContainerAllocator>;

  explicit Telemetry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->battery_mah_used = 0ul;
      this->rssi = 0;
      this->mode = "";
    }
  }

  explicit Telemetry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->battery_mah_used = 0ul;
      this->rssi = 0;
      this->mode = "";
    }
  }

  // field types and members
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _battery_mah_used_type =
    uint32_t;
  _battery_mah_used_type battery_mah_used;
  using _rssi_type =
    int8_t;
  _rssi_type rssi;
  using _mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__battery_mah_used(
    const uint32_t & _arg)
  {
    this->battery_mah_used = _arg;
    return *this;
  }
  Type & set__rssi(
    const int8_t & _arg)
  {
    this->rssi = _arg;
    return *this;
  }
  Type & set__mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Telemetry_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Telemetry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Telemetry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Telemetry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Telemetry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Telemetry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Telemetry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Telemetry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Telemetry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Telemetry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Telemetry
    std::shared_ptr<interfaces::msg::Telemetry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Telemetry
    std::shared_ptr<interfaces::msg::Telemetry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Telemetry_ & other) const
  {
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->battery_mah_used != other.battery_mah_used) {
      return false;
    }
    if (this->rssi != other.rssi) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const Telemetry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Telemetry_

// alias to use template instance with default allocator
using Telemetry =
  interfaces::msg::Telemetry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__TELEMETRY__STRUCT_HPP_
