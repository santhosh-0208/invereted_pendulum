// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from esp32_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef ESP32_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_
#define ESP32_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "esp32_msgs/msg/detail/imu_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace esp32_msgs
{

namespace msg
{

namespace builder
{

class Init_ImuData_az
{
public:
  explicit Init_ImuData_az(::esp32_msgs::msg::ImuData & msg)
  : msg_(msg)
  {}
  ::esp32_msgs::msg::ImuData az(::esp32_msgs::msg::ImuData::_az_type arg)
  {
    msg_.az = std::move(arg);
    return std::move(msg_);
  }

private:
  ::esp32_msgs::msg::ImuData msg_;
};

class Init_ImuData_ay
{
public:
  explicit Init_ImuData_ay(::esp32_msgs::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_az ay(::esp32_msgs::msg::ImuData::_ay_type arg)
  {
    msg_.ay = std::move(arg);
    return Init_ImuData_az(msg_);
  }

private:
  ::esp32_msgs::msg::ImuData msg_;
};

class Init_ImuData_ax
{
public:
  Init_ImuData_ax()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImuData_ay ax(::esp32_msgs::msg::ImuData::_ax_type arg)
  {
    msg_.ax = std::move(arg);
    return Init_ImuData_ay(msg_);
  }

private:
  ::esp32_msgs::msg::ImuData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::esp32_msgs::msg::ImuData>()
{
  return esp32_msgs::msg::builder::Init_ImuData_ax();
}

}  // namespace esp32_msgs

#endif  // ESP32_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_
