// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sample_interfaces:msg/TwoEncoderStatus.idl
// generated code does not contain a copyright notice
#include "sample_interfaces/msg/detail/two_encoder_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `left`
// Member `right`
#include "sample_interfaces/msg/detail/encoder_status__functions.h"

bool
sample_interfaces__msg__TwoEncoderStatus__init(sample_interfaces__msg__TwoEncoderStatus * msg)
{
  if (!msg) {
    return false;
  }
  // left
  if (!sample_interfaces__msg__EncoderStatus__init(&msg->left)) {
    sample_interfaces__msg__TwoEncoderStatus__fini(msg);
    return false;
  }
  // right
  if (!sample_interfaces__msg__EncoderStatus__init(&msg->right)) {
    sample_interfaces__msg__TwoEncoderStatus__fini(msg);
    return false;
  }
  return true;
}

void
sample_interfaces__msg__TwoEncoderStatus__fini(sample_interfaces__msg__TwoEncoderStatus * msg)
{
  if (!msg) {
    return;
  }
  // left
  sample_interfaces__msg__EncoderStatus__fini(&msg->left);
  // right
  sample_interfaces__msg__EncoderStatus__fini(&msg->right);
}

bool
sample_interfaces__msg__TwoEncoderStatus__are_equal(const sample_interfaces__msg__TwoEncoderStatus * lhs, const sample_interfaces__msg__TwoEncoderStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // left
  if (!sample_interfaces__msg__EncoderStatus__are_equal(
      &(lhs->left), &(rhs->left)))
  {
    return false;
  }
  // right
  if (!sample_interfaces__msg__EncoderStatus__are_equal(
      &(lhs->right), &(rhs->right)))
  {
    return false;
  }
  return true;
}

bool
sample_interfaces__msg__TwoEncoderStatus__copy(
  const sample_interfaces__msg__TwoEncoderStatus * input,
  sample_interfaces__msg__TwoEncoderStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // left
  if (!sample_interfaces__msg__EncoderStatus__copy(
      &(input->left), &(output->left)))
  {
    return false;
  }
  // right
  if (!sample_interfaces__msg__EncoderStatus__copy(
      &(input->right), &(output->right)))
  {
    return false;
  }
  return true;
}

sample_interfaces__msg__TwoEncoderStatus *
sample_interfaces__msg__TwoEncoderStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__TwoEncoderStatus * msg = (sample_interfaces__msg__TwoEncoderStatus *)allocator.allocate(sizeof(sample_interfaces__msg__TwoEncoderStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sample_interfaces__msg__TwoEncoderStatus));
  bool success = sample_interfaces__msg__TwoEncoderStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sample_interfaces__msg__TwoEncoderStatus__destroy(sample_interfaces__msg__TwoEncoderStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sample_interfaces__msg__TwoEncoderStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sample_interfaces__msg__TwoEncoderStatus__Sequence__init(sample_interfaces__msg__TwoEncoderStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__TwoEncoderStatus * data = NULL;

  if (size) {
    data = (sample_interfaces__msg__TwoEncoderStatus *)allocator.zero_allocate(size, sizeof(sample_interfaces__msg__TwoEncoderStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sample_interfaces__msg__TwoEncoderStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sample_interfaces__msg__TwoEncoderStatus__fini(&data[i - 1]);
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
sample_interfaces__msg__TwoEncoderStatus__Sequence__fini(sample_interfaces__msg__TwoEncoderStatus__Sequence * array)
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
      sample_interfaces__msg__TwoEncoderStatus__fini(&array->data[i]);
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

sample_interfaces__msg__TwoEncoderStatus__Sequence *
sample_interfaces__msg__TwoEncoderStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interfaces__msg__TwoEncoderStatus__Sequence * array = (sample_interfaces__msg__TwoEncoderStatus__Sequence *)allocator.allocate(sizeof(sample_interfaces__msg__TwoEncoderStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sample_interfaces__msg__TwoEncoderStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sample_interfaces__msg__TwoEncoderStatus__Sequence__destroy(sample_interfaces__msg__TwoEncoderStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sample_interfaces__msg__TwoEncoderStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sample_interfaces__msg__TwoEncoderStatus__Sequence__are_equal(const sample_interfaces__msg__TwoEncoderStatus__Sequence * lhs, const sample_interfaces__msg__TwoEncoderStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sample_interfaces__msg__TwoEncoderStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sample_interfaces__msg__TwoEncoderStatus__Sequence__copy(
  const sample_interfaces__msg__TwoEncoderStatus__Sequence * input,
  sample_interfaces__msg__TwoEncoderStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sample_interfaces__msg__TwoEncoderStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sample_interfaces__msg__TwoEncoderStatus * data =
      (sample_interfaces__msg__TwoEncoderStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sample_interfaces__msg__TwoEncoderStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sample_interfaces__msg__TwoEncoderStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sample_interfaces__msg__TwoEncoderStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
