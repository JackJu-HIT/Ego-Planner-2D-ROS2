// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rviz_car_sim:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__FUNCTIONS_H_
#define RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rviz_car_sim/msg/rosidl_generator_c__visibility_control.h"

#include "rviz_car_sim/msg/detail/current_pose__struct.h"

/// Initialize msg/CurrentPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rviz_car_sim__msg__CurrentPose
 * )) before or use
 * rviz_car_sim__msg__CurrentPose__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
bool
rviz_car_sim__msg__CurrentPose__init(rviz_car_sim__msg__CurrentPose * msg);

/// Finalize msg/CurrentPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
void
rviz_car_sim__msg__CurrentPose__fini(rviz_car_sim__msg__CurrentPose * msg);

/// Create msg/CurrentPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rviz_car_sim__msg__CurrentPose__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
rviz_car_sim__msg__CurrentPose *
rviz_car_sim__msg__CurrentPose__create();

/// Destroy msg/CurrentPose message.
/**
 * It calls
 * rviz_car_sim__msg__CurrentPose__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
void
rviz_car_sim__msg__CurrentPose__destroy(rviz_car_sim__msg__CurrentPose * msg);

/// Check for msg/CurrentPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
bool
rviz_car_sim__msg__CurrentPose__are_equal(const rviz_car_sim__msg__CurrentPose * lhs, const rviz_car_sim__msg__CurrentPose * rhs);

/// Copy a msg/CurrentPose message.
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
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
bool
rviz_car_sim__msg__CurrentPose__copy(
  const rviz_car_sim__msg__CurrentPose * input,
  rviz_car_sim__msg__CurrentPose * output);

/// Initialize array of msg/CurrentPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * rviz_car_sim__msg__CurrentPose__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
bool
rviz_car_sim__msg__CurrentPose__Sequence__init(rviz_car_sim__msg__CurrentPose__Sequence * array, size_t size);

/// Finalize array of msg/CurrentPose messages.
/**
 * It calls
 * rviz_car_sim__msg__CurrentPose__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
void
rviz_car_sim__msg__CurrentPose__Sequence__fini(rviz_car_sim__msg__CurrentPose__Sequence * array);

/// Create array of msg/CurrentPose messages.
/**
 * It allocates the memory for the array and calls
 * rviz_car_sim__msg__CurrentPose__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
rviz_car_sim__msg__CurrentPose__Sequence *
rviz_car_sim__msg__CurrentPose__Sequence__create(size_t size);

/// Destroy array of msg/CurrentPose messages.
/**
 * It calls
 * rviz_car_sim__msg__CurrentPose__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
void
rviz_car_sim__msg__CurrentPose__Sequence__destroy(rviz_car_sim__msg__CurrentPose__Sequence * array);

/// Check for msg/CurrentPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
bool
rviz_car_sim__msg__CurrentPose__Sequence__are_equal(const rviz_car_sim__msg__CurrentPose__Sequence * lhs, const rviz_car_sim__msg__CurrentPose__Sequence * rhs);

/// Copy an array of msg/CurrentPose messages.
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
ROSIDL_GENERATOR_C_PUBLIC_rviz_car_sim
bool
rviz_car_sim__msg__CurrentPose__Sequence__copy(
  const rviz_car_sim__msg__CurrentPose__Sequence * input,
  rviz_car_sim__msg__CurrentPose__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__FUNCTIONS_H_
