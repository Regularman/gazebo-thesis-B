// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:srv/SetArming.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__SET_ARMING__STRUCT_HPP_
#define INTERFACES__SRV__DETAIL__SET_ARMING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__SetArming_Request __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__SetArming_Request __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetArming_Request_
{
  using Type = SetArming_Request_<ContainerAllocator>;

  explicit SetArming_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arm = false;
    }
  }

  explicit SetArming_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arm = false;
    }
  }

  // field types and members
  using _arm_type =
    bool;
  _arm_type arm;

  // setters for named parameter idiom
  Type & set__arm(
    const bool & _arg)
  {
    this->arm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::SetArming_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::SetArming_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::SetArming_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::SetArming_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::SetArming_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::SetArming_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::SetArming_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::SetArming_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::SetArming_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::SetArming_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__SetArming_Request
    std::shared_ptr<interfaces::srv::SetArming_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__SetArming_Request
    std::shared_ptr<interfaces::srv::SetArming_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetArming_Request_ & other) const
  {
    if (this->arm != other.arm) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetArming_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetArming_Request_

// alias to use template instance with default allocator
using SetArming_Request =
  interfaces::srv::SetArming_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__SetArming_Response __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__SetArming_Response __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetArming_Response_
{
  using Type = SetArming_Response_<ContainerAllocator>;

  explicit SetArming_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetArming_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::SetArming_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::SetArming_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::SetArming_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::SetArming_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::SetArming_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::SetArming_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::SetArming_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::SetArming_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::SetArming_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::SetArming_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__SetArming_Response
    std::shared_ptr<interfaces::srv::SetArming_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__SetArming_Response
    std::shared_ptr<interfaces::srv::SetArming_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetArming_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetArming_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetArming_Response_

// alias to use template instance with default allocator
using SetArming_Response =
  interfaces::srv::SetArming_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces

namespace interfaces
{

namespace srv
{

struct SetArming
{
  using Request = interfaces::srv::SetArming_Request;
  using Response = interfaces::srv::SetArming_Response;
};

}  // namespace srv

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__SET_ARMING__STRUCT_HPP_
