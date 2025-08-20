// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from esp32_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef ESP32_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_
#define ESP32_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__esp32_msgs__msg__ImuData __attribute__((deprecated))
#else
# define DEPRECATED__esp32_msgs__msg__ImuData __declspec(deprecated)
#endif

namespace esp32_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuData_
{
  using Type = ImuData_<ContainerAllocator>;

  explicit ImuData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ax = 0;
      this->ay = 0;
      this->az = 0;
    }
  }

  explicit ImuData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ax = 0;
      this->ay = 0;
      this->az = 0;
    }
  }

  // field types and members
  using _ax_type =
    int16_t;
  _ax_type ax;
  using _ay_type =
    int16_t;
  _ay_type ay;
  using _az_type =
    int16_t;
  _az_type az;

  // setters for named parameter idiom
  Type & set__ax(
    const int16_t & _arg)
  {
    this->ax = _arg;
    return *this;
  }
  Type & set__ay(
    const int16_t & _arg)
  {
    this->ay = _arg;
    return *this;
  }
  Type & set__az(
    const int16_t & _arg)
  {
    this->az = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    esp32_msgs::msg::ImuData_<ContainerAllocator> *;
  using ConstRawPtr =
    const esp32_msgs::msg::ImuData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<esp32_msgs::msg::ImuData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<esp32_msgs::msg::ImuData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      esp32_msgs::msg::ImuData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<esp32_msgs::msg::ImuData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      esp32_msgs::msg::ImuData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<esp32_msgs::msg::ImuData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<esp32_msgs::msg::ImuData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<esp32_msgs::msg::ImuData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__esp32_msgs__msg__ImuData
    std::shared_ptr<esp32_msgs::msg::ImuData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__esp32_msgs__msg__ImuData
    std::shared_ptr<esp32_msgs::msg::ImuData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuData_ & other) const
  {
    if (this->ax != other.ax) {
      return false;
    }
    if (this->ay != other.ay) {
      return false;
    }
    if (this->az != other.az) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuData_

// alias to use template instance with default allocator
using ImuData =
  esp32_msgs::msg::ImuData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace esp32_msgs

#endif  // ESP32_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_
