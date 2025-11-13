// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/SetArming.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__SET_ARMING__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__SET_ARMING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/set_arming__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_SetArming_Request_arm
{
public:
  Init_SetArming_Request_arm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::SetArming_Request arm(::interfaces::srv::SetArming_Request::_arm_type arg)
  {
    msg_.arm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::SetArming_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::SetArming_Request>()
{
  return interfaces::srv::builder::Init_SetArming_Request_arm();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_SetArming_Response_message
{
public:
  explicit Init_SetArming_Response_message(::interfaces::srv::SetArming_Response & msg)
  : msg_(msg)
  {}
  ::interfaces::srv::SetArming_Response message(::interfaces::srv::SetArming_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::SetArming_Response msg_;
};

class Init_SetArming_Response_success
{
public:
  Init_SetArming_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetArming_Response_message success(::interfaces::srv::SetArming_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetArming_Response_message(msg_);
  }

private:
  ::interfaces::srv::SetArming_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::SetArming_Response>()
{
  return interfaces::srv::builder::Init_SetArming_Response_success();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__SET_ARMING__BUILDER_HPP_
