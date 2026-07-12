// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from rviz_car_sim:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rviz_car_sim/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "rviz_car_sim/msg/detail/current_pose__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace rviz_car_sim
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rviz_car_sim
cdr_serialize(
  const rviz_car_sim::msg::CurrentPose & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rviz_car_sim
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rviz_car_sim::msg::CurrentPose & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rviz_car_sim
get_serialized_size(
  const rviz_car_sim::msg::CurrentPose & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rviz_car_sim
max_serialized_size_CurrentPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rviz_car_sim

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rviz_car_sim
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rviz_car_sim, msg, CurrentPose)();

#ifdef __cplusplus
}
#endif

#endif  // RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
