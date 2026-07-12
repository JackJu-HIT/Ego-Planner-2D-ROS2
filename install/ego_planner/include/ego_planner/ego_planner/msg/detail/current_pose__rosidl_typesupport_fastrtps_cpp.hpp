// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ego_planner:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ego_planner/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ego_planner/msg/detail/current_pose__struct.hpp"

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

namespace ego_planner
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ego_planner
cdr_serialize(
  const ego_planner::msg::CurrentPose & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ego_planner
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ego_planner::msg::CurrentPose & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ego_planner
get_serialized_size(
  const ego_planner::msg::CurrentPose & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ego_planner
max_serialized_size_CurrentPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ego_planner

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ego_planner
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ego_planner, msg, CurrentPose)();

#ifdef __cplusplus
}
#endif

#endif  // EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
