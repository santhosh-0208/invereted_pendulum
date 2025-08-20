// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from esp32_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef ESP32_MSGS__MSG__DETAIL__IMU_DATA__FUNCTIONS_H_
#define ESP32_MSGS__MSG__DETAIL__IMU_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "esp32_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "esp32_msgs/msg/detail/imu_data__struct.h"

/// Initialize msg/ImuData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * esp32_msgs__msg__ImuData
 * )) before or use
 * esp32_msgs__msg__ImuData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
bool
esp32_msgs__msg__ImuData__init(esp32_msgs__msg__ImuData * msg);

/// Finalize msg/ImuData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
void
esp32_msgs__msg__ImuData__fini(esp32_msgs__msg__ImuData * msg);

/// Create msg/ImuData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * esp32_msgs__msg__ImuData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
esp32_msgs__msg__ImuData *
esp32_msgs__msg__ImuData__create();

/// Destroy msg/ImuData message.
/**
 * It calls
 * esp32_msgs__msg__ImuData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
void
esp32_msgs__msg__ImuData__destroy(esp32_msgs__msg__ImuData * msg);

/// Check for msg/ImuData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
bool
esp32_msgs__msg__ImuData__are_equal(const esp32_msgs__msg__ImuData * lhs, const esp32_msgs__msg__ImuData * rhs);

/// Copy a msg/ImuData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
bool
esp32_msgs__msg__ImuData__copy(
  const esp32_msgs__msg__ImuData * input,
  esp32_msgs__msg__ImuData * output);

/// Initialize array of msg/ImuData messages.
/**
 * It allocates the memory for the number of elements and calls
 * esp32_msgs__msg__ImuData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
bool
esp32_msgs__msg__ImuData__Sequence__init(esp32_msgs__msg__ImuData__Sequence * array, size_t size);

/// Finalize array of msg/ImuData messages.
/**
 * It calls
 * esp32_msgs__msg__ImuData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
void
esp32_msgs__msg__ImuData__Sequence__fini(esp32_msgs__msg__ImuData__Sequence * array);

/// Create array of msg/ImuData messages.
/**
 * It allocates the memory for the array and calls
 * esp32_msgs__msg__ImuData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
esp32_msgs__msg__ImuData__Sequence *
esp32_msgs__msg__ImuData__Sequence__create(size_t size);

/// Destroy array of msg/ImuData messages.
/**
 * It calls
 * esp32_msgs__msg__ImuData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
void
esp32_msgs__msg__ImuData__Sequence__destroy(esp32_msgs__msg__ImuData__Sequence * array);

/// Check for msg/ImuData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
bool
esp32_msgs__msg__ImuData__Sequence__are_equal(const esp32_msgs__msg__ImuData__Sequence * lhs, const esp32_msgs__msg__ImuData__Sequence * rhs);

/// Copy an array of msg/ImuData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_esp32_msgs
bool
esp32_msgs__msg__ImuData__Sequence__copy(
  const esp32_msgs__msg__ImuData__Sequence * input,
  esp32_msgs__msg__ImuData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ESP32_MSGS__MSG__DETAIL__IMU_DATA__FUNCTIONS_H_
