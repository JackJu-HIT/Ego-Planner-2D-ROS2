// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ego_planner:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__STRUCT_H_
#define EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CurrentPose in the package ego_planner.
typedef struct ego_planner__msg__CurrentPose
{
  double x;
  double y;
  double theta;
} ego_planner__msg__CurrentPose;

// Struct for a sequence of ego_planner__msg__CurrentPose.
typedef struct ego_planner__msg__CurrentPose__Sequence
{
  ego_planner__msg__CurrentPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ego_planner__msg__CurrentPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__STRUCT_H_
