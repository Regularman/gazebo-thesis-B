// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/InvertedPendulumStates.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__INVERTED_PENDULUM_STATES__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__INVERTED_PENDULUM_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/inverted_pendulum_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_InvertedPendulumStates_twist
{
public:
  explicit Init_InvertedPendulumStates_twist(::interfaces::msg::InvertedPendulumStates & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::InvertedPendulumStates twist(::interfaces::msg::InvertedPendulumStates::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::InvertedPendulumStates msg_;
};

class Init_InvertedPendulumStates_pose
{
public:
  explicit Init_InvertedPendulumStates_pose(::interfaces::msg::InvertedPendulumStates & msg)
  : msg_(msg)
  {}
  Init_InvertedPendulumStates_twist pose(::interfaces::msg::InvertedPendulumStates::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_InvertedPendulumStates_twist(msg_);
  }

private:
  ::interfaces::msg::InvertedPendulumStates msg_;
};

class Init_InvertedPendulumStates_child_frame_id
{
public:
  explicit Init_InvertedPendulumStates_child_frame_id(::interfaces::msg::InvertedPendulumStates & msg)
  : msg_(msg)
  {}
  Init_InvertedPendulumStates_pose child_frame_id(::interfaces::msg::InvertedPendulumStates::_child_frame_id_type arg)
  {
    msg_.child_frame_id = std::move(arg);
    return Init_InvertedPendulumStates_pose(msg_);
  }

private:
  ::interfaces::msg::InvertedPendulumStates msg_;
};

class Init_InvertedPendulumStates_header
{
public:
  Init_InvertedPendulumStates_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InvertedPendulumStates_child_frame_id header(::interfaces::msg::InvertedPendulumStates::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_InvertedPendulumStates_child_frame_id(msg_);
  }

private:
  ::interfaces::msg::InvertedPendulumStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::InvertedPendulumStates>()
{
  return interfaces::msg::builder::Init_InvertedPendulumStates_header();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__INVERTED_PENDULUM_STATES__BUILDER_HPP_
