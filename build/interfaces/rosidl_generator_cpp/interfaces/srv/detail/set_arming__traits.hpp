// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:srv/SetArming.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__SET_ARMING__TRAITS_HPP_
#define INTERFACES__SRV__DETAIL__SET_ARMING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/srv/detail/set_arming__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetArming_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: arm
  {
    out << "arm: ";
    rosidl_generator_traits::value_to_yaml(msg.arm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetArming_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: arm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm: ";
    rosidl_generator_traits::value_to_yaml(msg.arm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetArming_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::SetArming_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::SetArming_Request & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::SetArming_Request>()
{
  return "interfaces::srv::SetArming_Request";
}

template<>
inline const char * name<interfaces::srv::SetArming_Request>()
{
  return "interfaces/srv/SetArming_Request";
}

template<>
struct has_fixed_size<interfaces::srv::SetArming_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::srv::SetArming_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::srv::SetArming_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetArming_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetArming_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetArming_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::SetArming_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::SetArming_Response & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::SetArming_Response>()
{
  return "interfaces::srv::SetArming_Response";
}

template<>
inline const char * name<interfaces::srv::SetArming_Response>()
{
  return "interfaces/srv/SetArming_Response";
}

template<>
struct has_fixed_size<interfaces::srv::SetArming_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::srv::SetArming_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::srv::SetArming_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::srv::SetArming>()
{
  return "interfaces::srv::SetArming";
}

template<>
inline const char * name<interfaces::srv::SetArming>()
{
  return "interfaces/srv/SetArming";
}

template<>
struct has_fixed_size<interfaces::srv::SetArming>
  : std::integral_constant<
    bool,
    has_fixed_size<interfaces::srv::SetArming_Request>::value &&
    has_fixed_size<interfaces::srv::SetArming_Response>::value
  >
{
};

template<>
struct has_bounded_size<interfaces::srv::SetArming>
  : std::integral_constant<
    bool,
    has_bounded_size<interfaces::srv::SetArming_Request>::value &&
    has_bounded_size<interfaces::srv::SetArming_Response>::value
  >
{
};

template<>
struct is_service<interfaces::srv::SetArming>
  : std::true_type
{
};

template<>
struct is_service_request<interfaces::srv::SetArming_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interfaces::srv::SetArming_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__SRV__DETAIL__SET_ARMING__TRAITS_HPP_
