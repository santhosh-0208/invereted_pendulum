// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from esp32_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef ESP32_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_
#define ESP32_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ImuData in the package esp32_msgs.
typedef struct esp32_msgs__msg__ImuData
{
  int16_t ax;
  int16_t ay;
  int16_t az;
} esp32_msgs__msg__ImuData;

// Struct for a sequence of esp32_msgs__msg__ImuData.
typedef struct esp32_msgs__msg__ImuData__Sequence
{
  esp32_msgs__msg__ImuData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} esp32_msgs__msg__ImuData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ESP32_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_
