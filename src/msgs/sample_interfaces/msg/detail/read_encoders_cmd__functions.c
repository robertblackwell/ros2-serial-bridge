// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sample_interfaces:msg/ReadEncodersCmd.idl
// generated code does not contain a copyright notice
#include "sample_interfaces/msg/detail/read_encoders_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
sample_interfaces__msg__ReadEncodersCmd__init(sample_interfaces__msg__ReadEncodersCmd * msg)
{
  if (!msg) {
    return false;
  }
  // n
  return true;
}

void
sample_interfaces__msg__ReadEncodersCmd__fini(sample_interfaces__msg__ReadEncodersCmd * msg)
{
  if (!msg) {
    return;
  }
  // n
}

bool
sample_interfaces__msg__ReadEncodersCmd__are_equal(const sample_interfaces__msg__ReadEncodersCmd * lhs, const sample_interfaces__msg__ReadEncodersCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // n
  if (lhs->n != rhs->n) {
    return false;
  }
  return true;
}

bool
sample_interfaces__msg__ReadEncodersCmd__copy(
  const sample_interfaces__msg__ReadEncodersCmd * input,
  sample_interfaces__msg__ReadEncodersCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // n
  output->n = input->n;
  return true;
}

sample_interfaces__msg__ReadEncodersCmd *
sample_interfaces__msg__ReadEncodersCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__ReadEncodersCmd * msg = (sample_interfaces__msg__ReadEncodersCmd *)allocator.allocate(sizeof(sample_interfaces__msg__ReadEncodersCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sample_interfaces__msg__ReadEncodersCmd));
  bool success = sample_interfaces__msg__ReadEncodersCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sample_interfaces__msg__ReadEncodersCmd__destroy(sample_interfaces__msg__ReadEncodersCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sample_interfaces__msg__ReadEncodersCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sample_interfaces__msg__ReadEncodersCmd__Sequence__init(sample_interfaces__msg__ReadEncodersCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__ReadEncodersCmd * data = NULL;

  if (size) {
    data = (sample_interfaces__msg__ReadEncodersCmd *)allocator.zero_allocate(size, sizeof(sample_interfaces__msg__ReadEncodersCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sample_interfaces__msg__ReadEncodersCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sample_interfaces__msg__ReadEncodersCmd__fini(&data[i - 1]);
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
sample_interfaces__msg__ReadEncodersCmd__Sequence__fini(sample_interfaces__msg__ReadEncodersCmd__Sequence * array)
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
      sample_interfaces__msg__ReadEncodersCmd__fini(&array->data[i]);
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

sample_interfaces__msg__ReadEncodersCmd__Sequence *
sample_interfaces__msg__ReadEncodersCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__ReadEncodersCmd__Sequence * array = (sample_interfaces__msg__ReadEncodersCmd__Sequence *)allocator.allocate(sizeof(sample_interfaces__msg__ReadEncodersCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sample_interfaces__msg__ReadEncodersCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sample_interfaces__msg__ReadEncodersCmd__Sequence__destroy(sample_interfaces__msg__ReadEncodersCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sample_interfaces__msg__ReadEncodersCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sample_interfaces__msg__ReadEncodersCmd__Sequence__are_equal(const sample_interfaces__msg__ReadEncodersCmd__Sequence * lhs, const sample_interfaces__msg__ReadEncodersCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sample_interfaces__msg__ReadEncodersCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sample_interfaces__msg__ReadEncodersCmd__Sequence__copy(
  const sample_interfaces__msg__ReadEncodersCmd__Sequence * input,
  sample_interfaces__msg__ReadEncodersCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sample_interfaces__msg__ReadEncodersCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sample_interfaces__msg__ReadEncodersCmd * data =
      (sample_interfaces__msg__ReadEncodersCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sample_interfaces__msg__ReadEncodersCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sample_interfaces__msg__ReadEncodersCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sample_interfaces__msg__ReadEncodersCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
