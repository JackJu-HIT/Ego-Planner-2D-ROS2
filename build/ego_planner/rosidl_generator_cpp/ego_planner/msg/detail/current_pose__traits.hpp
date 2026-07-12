// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ego_planner:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__TRAITS_HPP_
#define EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ego_planner/msg/detail/current_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ego_planner
{

namespace msg
{

inline void to_flow_style_yaml(
  const CurrentPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CurrentPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CurrentPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ego_planner

namespace rosidl_generator_traits
{

[[deprecated("use ego_planner::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ego_planner::msg::CurrentPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  ego_planner::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ego_planner::msg::to_yaml() instead")]]
inline std::string to_yaml(const ego_planner::msg::CurrentPose & msg)
{
  return ego_planner::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ego_planner::msg::CurrentPose>()
{
  return "ego_planner::msg::CurrentPose";
}

template<>
inline const char * name<ego_planner::msg::CurrentPose>()
{
  return "ego_planner/msg/CurrentPose";
}

template<>
struct has_fixed_size<ego_planner::msg::CurrentPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ego_planner::msg::CurrentPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ego_planner::msg::CurrentPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__TRAITS_HPP_
