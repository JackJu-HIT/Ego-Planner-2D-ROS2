// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rviz_car_sim:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__TRAITS_HPP_
#define RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rviz_car_sim/msg/detail/current_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rviz_car_sim
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

}  // namespace rviz_car_sim

namespace rosidl_generator_traits
{

[[deprecated("use rviz_car_sim::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rviz_car_sim::msg::CurrentPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  rviz_car_sim::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rviz_car_sim::msg::to_yaml() instead")]]
inline std::string to_yaml(const rviz_car_sim::msg::CurrentPose & msg)
{
  return rviz_car_sim::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rviz_car_sim::msg::CurrentPose>()
{
  return "rviz_car_sim::msg::CurrentPose";
}

template<>
inline const char * name<rviz_car_sim::msg::CurrentPose>()
{
  return "rviz_car_sim/msg/CurrentPose";
}

template<>
struct has_fixed_size<rviz_car_sim::msg::CurrentPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rviz_car_sim::msg::CurrentPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rviz_car_sim::msg::CurrentPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__TRAITS_HPP_
