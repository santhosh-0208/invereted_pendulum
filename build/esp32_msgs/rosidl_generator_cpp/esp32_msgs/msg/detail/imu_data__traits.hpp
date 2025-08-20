// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from esp32_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef ESP32_MSGS__MSG__DETAIL__IMU_DATA__TRAITS_HPP_
#define ESP32_MSGS__MSG__DETAIL__IMU_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "esp32_msgs/msg/detail/imu_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace esp32_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ImuData & msg,
  std::ostream & out)
{
  out << "{";
  // member: ax
  {
    out << "ax: ";
    rosidl_generator_traits::value_to_yaml(msg.ax, out);
    out << ", ";
  }

  // member: ay
  {
    out << "ay: ";
    rosidl_generator_traits::value_to_yaml(msg.ay, out);
    out << ", ";
  }

  // member: az
  {
    out << "az: ";
    rosidl_generator_traits::value_to_yaml(msg.az, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImuData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ax
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ax: ";
    rosidl_generator_traits::value_to_yaml(msg.ax, out);
    out << "\n";
  }

  // member: ay
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ay: ";
    rosidl_generator_traits::value_to_yaml(msg.ay, out);
    out << "\n";
  }

  // member: az
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "az: ";
    rosidl_generator_traits::value_to_yaml(msg.az, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImuData & msg, bool use_flow_style = false)
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

}  // namespace esp32_msgs

namespace rosidl_generator_traits
{

[[deprecated("use esp32_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const esp32_msgs::msg::ImuData & msg,
  std::ostream & out, size_t indentation = 0)
{
  esp32_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use esp32_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const esp32_msgs::msg::ImuData & msg)
{
  return esp32_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<esp32_msgs::msg::ImuData>()
{
  return "esp32_msgs::msg::ImuData";
}

template<>
inline const char * name<esp32_msgs::msg::ImuData>()
{
  return "esp32_msgs/msg/ImuData";
}

template<>
struct has_fixed_size<esp32_msgs::msg::ImuData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<esp32_msgs::msg::ImuData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<esp32_msgs::msg::ImuData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ESP32_MSGS__MSG__DETAIL__IMU_DATA__TRAITS_HPP_
