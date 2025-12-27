// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sample_interfaces:msg/LoadTestCmd.idl
// generated code does not contain a copyright notice
#include "sample_interfaces/msg/detail/load_test_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
sample_interfaces__msg__LoadTestCmd__init(sample_interfaces__msg__LoadTestCmd * msg)
{
  if (!msg) {
    return false;
  }
  // count
  // msg_length
  // msgs_per_seecond
  return true;
}

void
sample_interfaces__msg__LoadTestCmd__fini(sample_interfaces__msg__LoadTestCmd * msg)
{
  if (!msg) {
    return;
  }
  // count
  // msg_length
  // msgs_per_seecond
}

bool
sample_interfaces__msg__LoadTestCmd__are_equal(const sample_interfaces__msg__LoadTestCmd * lhs, const sample_interfaces__msg__LoadTestCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // count
  if (lhs->count != rhs->count) {
    return false;
  }
  // msg_length
  if (lhs->msg_length != rhs->msg_length) {
    return false;
  }
  // msgs_per_seecond
  if (lhs->msgs_per_seecond != rhs->msgs_per_seecond) {
    return false;
  }
  return true;
}

bool
sample_interfaces__msg__LoadTestCmd__copy(
  const sample_interfaces__msg__LoadTestCmd * input,
  sample_interfaces__msg__LoadTestCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // count
  output->count = input->count;
  // msg_length
  output->msg_length = input->msg_length;
  // msgs_per_seecond
  output->msgs_per_seecond = input->msgs_per_seecond;
  return true;
}

sample_interfaces__msg__LoadTestCmd *
sample_interfaces__msg__LoadTestCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__LoadTestCmd * msg = (sample_interfaces__msg__LoadTestCmd *)allocator.allocate(sizeof(sample_interfaces__msg__LoadTestCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sample_interfaces__msg__LoadTestCmd));
  bool success = sample_interfaces__msg__LoadTestCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sample_interfaces__msg__LoadTestCmd__destroy(sample_interfaces__msg__LoadTestCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sample_interfaces__msg__LoadTestCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sample_interfaces__msg__LoadTestCmd__Sequence__init(sample_interfaces__msg__LoadTestCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__LoadTestCmd * data = NULL;

  if (size) {
    data = (sample_interfaces__msg__LoadTestCmd *)allocator.zero_allocate(size, sizeof(sample_interfaces__msg__LoadTestCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sample_interfaces__msg__LoadTestCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sample_interfaces__msg__LoadTestCmd__fini(&data[i - 1]);
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
sample_interfaces__msg__LoadTestCmd__Sequence__fini(sample_interfaces__msg__LoadTestCmd__Sequence * array)
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
      sample_interfaces__msg__LoadTestCmd__fini(&array->data[i]);
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

sample_interfaces__msg__LoadTestCmd__Sequence *
sample_interfaces__msg__LoadTestCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__LoadTestCmd__Sequence * array = (sample_interfaces__msg__LoadTestCmd__Sequence *)allocator.allocate(sizeof(sample_interfaces__msg__LoadTestCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sample_interfaces__msg__LoadTestCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sample_interfaces__msg__LoadTestCmd__Sequence__destroy(sample_interfaces__msg__LoadTestCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sample_interfaces__msg__LoadTestCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sample_interfaces__msg__LoadTestCmd__Sequence__are_equal(const sample_interfaces__msg__LoadTestCmd__Sequence * lhs, const sample_interfaces__msg__LoadTestCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sample_interfaces__msg__LoadTestCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sample_interfaces__msg__LoadTestCmd__Sequence__copy(
  const sample_interfaces__msg__LoadTestCmd__Sequence * input,
  sample_interfaces__msg__LoadTestCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sample_interfaces__msg__LoadTestCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sample_interfaces__msg__LoadTestCmd * data =
      (sample_interfaces__msg__LoadTestCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sample_interfaces__msg__LoadTestCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sample_interfaces__msg__LoadTestCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sample_interfaces__msg__LoadTestCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
