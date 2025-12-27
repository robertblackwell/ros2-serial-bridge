// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sample_interfaces:msg/CmdResponse.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__FUNCTIONS_H_
#define SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "sample_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "sample_interfaces/msg/detail/cmd_response__struct.h"

/// Initialize msg/CmdResponse message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sample_interfaces__msg__CmdResponse
 * )) before or use
 * sample_interfaces__msg__CmdResponse__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
bool
sample_interfaces__msg__CmdResponse__init(sample_interfaces__msg__CmdResponse * msg);

/// Finalize msg/CmdResponse message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
void
sample_interfaces__msg__CmdResponse__fini(sample_interfaces__msg__CmdResponse * msg);

/// Create msg/CmdResponse message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sample_interfaces__msg__CmdResponse__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
sample_interfaces__msg__CmdResponse *
sample_interfaces__msg__CmdResponse__create();

/// Destroy msg/CmdResponse message.
/**
 * It calls
 * sample_interfaces__msg__CmdResponse__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
void
sample_interfaces__msg__CmdResponse__destroy(sample_interfaces__msg__CmdResponse * msg);

/// Check for msg/CmdResponse message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
bool
sample_interfaces__msg__CmdResponse__are_equal(const sample_interfaces__msg__CmdResponse * lhs, const sample_interfaces__msg__CmdResponse * rhs);

/// Copy a msg/CmdResponse message.
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
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
bool
sample_interfaces__msg__CmdResponse__copy(
  const sample_interfaces__msg__CmdResponse * input,
  sample_interfaces__msg__CmdResponse * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_type_hash_t *
sample_interfaces__msg__CmdResponse__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
sample_interfaces__msg__CmdResponse__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_runtime_c__type_description__TypeSource *
sample_interfaces__msg__CmdResponse__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
sample_interfaces__msg__CmdResponse__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/CmdResponse messages.
/**
 * It allocates the memory for the number of elements and calls
 * sample_interfaces__msg__CmdResponse__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
bool
sample_interfaces__msg__CmdResponse__Sequence__init(sample_interfaces__msg__CmdResponse__Sequence * array, size_t size);

/// Finalize array of msg/CmdResponse messages.
/**
 * It calls
 * sample_interfaces__msg__CmdResponse__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
void
sample_interfaces__msg__CmdResponse__Sequence__fini(sample_interfaces__msg__CmdResponse__Sequence * array);

/// Create array of msg/CmdResponse messages.
/**
 * It allocates the memory for the array and calls
 * sample_interfaces__msg__CmdResponse__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
sample_interfaces__msg__CmdResponse__Sequence *
sample_interfaces__msg__CmdResponse__Sequence__create(size_t size);

/// Destroy array of msg/CmdResponse messages.
/**
 * It calls
 * sample_interfaces__msg__CmdResponse__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
void
sample_interfaces__msg__CmdResponse__Sequence__destroy(sample_interfaces__msg__CmdResponse__Sequence * array);

/// Check for msg/CmdResponse message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
bool
sample_interfaces__msg__CmdResponse__Sequence__are_equal(const sample_interfaces__msg__CmdResponse__Sequence * lhs, const sample_interfaces__msg__CmdResponse__Sequence * rhs);

/// Copy an array of msg/CmdResponse messages.
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
ROSIDL_GENERATOR_C_PUBLIC_sample_interfaces
bool
sample_interfaces__msg__CmdResponse__Sequence__copy(
  const sample_interfaces__msg__CmdResponse__Sequence * input,
  sample_interfaces__msg__CmdResponse__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__CMD_RESPONSE__FUNCTIONS_H_
