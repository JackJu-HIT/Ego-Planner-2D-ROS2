// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rviz_car_sim:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__STRUCT_H_
#define RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CurrentPose in the package rviz_car_sim.
typedef struct rviz_car_sim__msg__CurrentPose
{
  double x;
  double y;
  double theta;
} rviz_car_sim__msg__CurrentPose;

// Struct for a sequence of rviz_car_sim__msg__CurrentPose.
typedef struct rviz_car_sim__msg__CurrentPose__Sequence
{
  rviz_car_sim__msg__CurrentPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rviz_car_sim__msg__CurrentPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__STRUCT_H_
