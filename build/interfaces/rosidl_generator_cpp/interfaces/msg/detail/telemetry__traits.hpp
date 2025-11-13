// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/Telemetry.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__TELEMETRY__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__TELEMETRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/telemetry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Telemetry & msg,
  std::ostream & out)
{
  out << "{";
  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: battery_mah_used
  {
    out << "battery_mah_used: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_mah_used, out);
    out << ", ";
  }

  // member: rssi
  {
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Telemetry & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: battery_mah_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_mah_used: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_mah_used, out);
    out << "\n";
  }

  // member: rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Telemetry & msg, bool use_flow_style = false)
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
  const interfaces::msg::Telemetry & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::Telemetry & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::Telemetry>()
{
  return "interfaces::msg::Telemetry";
}

template<>
inline const char * name<interfaces::msg::Telemetry>()
{
  return "interfaces/msg/Telemetry";
}

template<>
struct has_fixed_size<interfaces::msg::Telemetry>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::Telemetry>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::Telemetry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__TELEMETRY__TRAITS_HPP_
