// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/ControlApplied.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CONTROL_APPLIED__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__CONTROL_APPLIED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/control_applied__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControlApplied & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: pose
  {
    if (msg.pose.size() == 0) {
      out << "pose: []";
    } else {
      out << "pose: [";
      size_t pending_items = msg.pose.size();
      for (auto item : msg.pose) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: u
  {
    if (msg.u.size() == 0) {
      out << "u: []";
    } else {
      out << "u: [";
      size_t pending_items = msg.u.size();
      for (auto item : msg.u) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: u_rate
  {
    if (msg.u_rate.size() == 0) {
      out << "u_rate: []";
    } else {
      out << "u_rate: [";
      size_t pending_items = msg.u_rate.size();
      for (auto item : msg.u_rate) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: est_params
  {
    if (msg.est_params.size() == 0) {
      out << "est_params: []";
    } else {
      out << "est_params: [";
      size_t pending_items = msg.est_params.size();
      for (auto item : msg.est_params) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlApplied & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose.size() == 0) {
      out << "pose: []\n";
    } else {
      out << "pose:\n";
      for (auto item : msg.pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: u
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.u.size() == 0) {
      out << "u: []\n";
    } else {
      out << "u:\n";
      for (auto item : msg.u) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: u_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.u_rate.size() == 0) {
      out << "u_rate: []\n";
    } else {
      out << "u_rate:\n";
      for (auto item : msg.u_rate) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: est_params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.est_params.size() == 0) {
      out << "est_params: []\n";
    } else {
      out << "est_params:\n";
      for (auto item : msg.est_params) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlApplied & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::ControlApplied & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::ControlApplied & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::ControlApplied>()
{
  return "interfaces::msg::ControlApplied";
}

template<>
inline const char * name<interfaces::msg::ControlApplied>()
{
  return "interfaces/msg/ControlApplied";
}

template<>
struct has_fixed_size<interfaces::msg::ControlApplied>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<interfaces::msg::ControlApplied>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<interfaces::msg::ControlApplied>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__CONTROL_APPLIED__TRAITS_HPP_
