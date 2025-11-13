// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Telemetry.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__TELEMETRY__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__TELEMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/telemetry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Telemetry_mode
{
public:
  explicit Init_Telemetry_mode(::interfaces::msg::Telemetry & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Telemetry mode(::interfaces::msg::Telemetry::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Telemetry msg_;
};

class Init_Telemetry_rssi
{
public:
  explicit Init_Telemetry_rssi(::interfaces::msg::Telemetry & msg)
  : msg_(msg)
  {}
  Init_Telemetry_mode rssi(::interfaces::msg::Telemetry::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return Init_Telemetry_mode(msg_);
  }

private:
  ::interfaces::msg::Telemetry msg_;
};

class Init_Telemetry_battery_mah_used
{
public:
  explicit Init_Telemetry_battery_mah_used(::interfaces::msg::Telemetry & msg)
  : msg_(msg)
  {}
  Init_Telemetry_rssi battery_mah_used(::interfaces::msg::Telemetry::_battery_mah_used_type arg)
  {
    msg_.battery_mah_used = std::move(arg);
    return Init_Telemetry_rssi(msg_);
  }

private:
  ::interfaces::msg::Telemetry msg_;
};

class Init_Telemetry_battery_voltage
{
public:
  Init_Telemetry_battery_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Telemetry_battery_mah_used battery_voltage(::interfaces::msg::Telemetry::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_Telemetry_battery_mah_used(msg_);
  }

private:
  ::interfaces::msg::Telemetry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Telemetry>()
{
  return interfaces::msg::builder::Init_Telemetry_battery_voltage();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__TELEMETRY__BUILDER_HPP_
