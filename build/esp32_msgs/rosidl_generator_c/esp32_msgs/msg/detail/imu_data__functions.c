// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from esp32_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice
#include "esp32_msgs/msg/detail/imu_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
esp32_msgs__msg__ImuData__init(esp32_msgs__msg__ImuData * msg)
{
  if (!msg) {
    return false;
  }
  // ax
  // ay
  // az
  return true;
}

void
esp32_msgs__msg__ImuData__fini(esp32_msgs__msg__ImuData * msg)
{
  if (!msg) {
    return;
  }
  // ax
  // ay
  // az
}

bool
esp32_msgs__msg__ImuData__are_equal(const esp32_msgs__msg__ImuData * lhs, const esp32_msgs__msg__ImuData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ax
  if (lhs->ax != rhs->ax) {
    return false;
  }
  // ay
  if (lhs->ay != rhs->ay) {
    return false;
  }
  // az
  if (lhs->az != rhs->az) {
    return false;
  }
  return true;
}

bool
esp32_msgs__msg__ImuData__copy(
  const esp32_msgs__msg__ImuData * input,
  esp32_msgs__msg__ImuData * output)
{
  if (!input || !output) {
    return false;
  }
  // ax
  output->ax = input->ax;
  // ay
  output->ay = input->ay;
  // az
  output->az = input->az;
  return true;
}

esp32_msgs__msg__ImuData *
esp32_msgs__msg__ImuData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  esp32_msgs__msg__ImuData * msg = (esp32_msgs__msg__ImuData *)allocator.allocate(sizeof(esp32_msgs__msg__ImuData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(esp32_msgs__msg__ImuData));
  bool success = esp32_msgs__msg__ImuData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
esp32_msgs__msg__ImuData__destroy(esp32_msgs__msg__ImuData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    esp32_msgs__msg__ImuData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
esp32_msgs__msg__ImuData__Sequence__init(esp32_msgs__msg__ImuData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  esp32_msgs__msg__ImuData * data = NULL;

  if (size) {
    data = (esp32_msgs__msg__ImuData *)allocator.zero_allocate(size, sizeof(esp32_msgs__msg__ImuData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = esp32_msgs__msg__ImuData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        esp32_msgs__msg__ImuData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
esp32_msgs__msg__ImuData__Sequence__fini(esp32_msgs__msg__ImuData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      esp32_msgs__msg__ImuData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

esp32_msgs__msg__ImuData__Sequence *
esp32_msgs__msg__ImuData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  esp32_msgs__msg__ImuData__Sequence * array = (esp32_msgs__msg__ImuData__Sequence *)allocator.allocate(sizeof(esp32_msgs__msg__ImuData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = esp32_msgs__msg__ImuData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
esp32_msgs__msg__ImuData__Sequence__destroy(esp32_msgs__msg__ImuData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    esp32_msgs__msg__ImuData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
esp32_msgs__msg__ImuData__Sequence__are_equal(const esp32_msgs__msg__ImuData__Sequence * lhs, const esp32_msgs__msg__ImuData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!esp32_msgs__msg__ImuData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
esp32_msgs__msg__ImuData__Sequence__copy(
  const esp32_msgs__msg__ImuData__Sequence * input,
  esp32_msgs__msg__ImuData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(esp32_msgs__msg__ImuData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    esp32_msgs__msg__ImuData * data =
      (esp32_msgs__msg__ImuData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!esp32_msgs__msg__ImuData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          esp32_msgs__msg__ImuData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!esp32_msgs__msg__ImuData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
