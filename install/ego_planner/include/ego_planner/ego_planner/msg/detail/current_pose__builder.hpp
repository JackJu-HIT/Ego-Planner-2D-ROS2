// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ego_planner:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__BUILDER_HPP_
#define EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ego_planner/msg/detail/current_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ego_planner
{

namespace msg
{

namespace builder
{

class Init_CurrentPose_theta
{
public:
  explicit Init_CurrentPose_theta(::ego_planner::msg::CurrentPose & msg)
  : msg_(msg)
  {}
  ::ego_planner::msg::CurrentPose theta(::ego_planner::msg::CurrentPose::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ego_planner::msg::CurrentPose msg_;
};

class Init_CurrentPose_y
{
public:
  explicit Init_CurrentPose_y(::ego_planner::msg::CurrentPose & msg)
  : msg_(msg)
  {}
  Init_CurrentPose_theta y(::ego_planner::msg::CurrentPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CurrentPose_theta(msg_);
  }

private:
  ::ego_planner::msg::CurrentPose msg_;
};

class Init_CurrentPose_x
{
public:
  Init_CurrentPose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CurrentPose_y x(::ego_planner::msg::CurrentPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CurrentPose_y(msg_);
  }

private:
  ::ego_planner::msg::CurrentPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ego_planner::msg::CurrentPose>()
{
  return ego_planner::msg::builder::Init_CurrentPose_x();
}

}  // namespace ego_planner

#endif  // EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__BUILDER_HPP_
