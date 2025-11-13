// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/ControlApplied.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CONTROL_APPLIED__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__CONTROL_APPLIED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/control_applied__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_ControlApplied_est_params
{
public:
  explicit Init_ControlApplied_est_params(::interfaces::msg::ControlApplied & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::ControlApplied est_params(::interfaces::msg::ControlApplied::_est_params_type arg)
  {
    msg_.est_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::ControlApplied msg_;
};

class Init_ControlApplied_u_rate
{
public:
  explicit Init_ControlApplied_u_rate(::interfaces::msg::ControlApplied & msg)
  : msg_(msg)
  {}
  Init_ControlApplied_est_params u_rate(::interfaces::msg::ControlApplied::_u_rate_type arg)
  {
    msg_.u_rate = std::move(arg);
    return Init_ControlApplied_est_params(msg_);
  }

private:
  ::interfaces::msg::ControlApplied msg_;
};

class Init_ControlApplied_u
{
public:
  explicit Init_ControlApplied_u(::interfaces::msg::ControlApplied & msg)
  : msg_(msg)
  {}
  Init_ControlApplied_u_rate u(::interfaces::msg::ControlApplied::_u_type arg)
  {
    msg_.u = std::move(arg);
    return Init_ControlApplied_u_rate(msg_);
  }

private:
  ::interfaces::msg::ControlApplied msg_;
};

class Init_ControlApplied_pose
{
public:
  explicit Init_ControlApplied_pose(::interfaces::msg::ControlApplied & msg)
  : msg_(msg)
  {}
  Init_ControlApplied_u pose(::interfaces::msg::ControlApplied::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ControlApplied_u(msg_);
  }

private:
  ::interfaces::msg::ControlApplied msg_;
};

class Init_ControlApplied_stamp
{
public:
  Init_ControlApplied_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlApplied_pose stamp(::interfaces::msg::ControlApplied::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_ControlApplied_pose(msg_);
  }

private:
  ::interfaces::msg::ControlApplied msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::ControlApplied>()
{
  return interfaces::msg::builder::Init_ControlApplied_stamp();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CONTROL_APPLIED__BUILDER_HPP_
