// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rviz_car_sim:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__BUILDER_HPP_
#define RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rviz_car_sim/msg/detail/current_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rviz_car_sim
{

namespace msg
{

namespace builder
{

class Init_CurrentPose_theta
{
public:
  explicit Init_CurrentPose_theta(::rviz_car_sim::msg::CurrentPose & msg)
  : msg_(msg)
  {}
  ::rviz_car_sim::msg::CurrentPose theta(::rviz_car_sim::msg::CurrentPose::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rviz_car_sim::msg::CurrentPose msg_;
};

class Init_CurrentPose_y
{
public:
  explicit Init_CurrentPose_y(::rviz_car_sim::msg::CurrentPose & msg)
  : msg_(msg)
  {}
  Init_CurrentPose_theta y(::rviz_car_sim::msg::CurrentPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CurrentPose_theta(msg_);
  }

private:
  ::rviz_car_sim::msg::CurrentPose msg_;
};

class Init_CurrentPose_x
{
public:
  Init_CurrentPose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CurrentPose_y x(::rviz_car_sim::msg::CurrentPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CurrentPose_y(msg_);
  }

private:
  ::rviz_car_sim::msg::CurrentPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rviz_car_sim::msg::CurrentPose>()
{
  return rviz_car_sim::msg::builder::Init_CurrentPose_x();
}

}  // namespace rviz_car_sim

#endif  // RVIZ_CAR_SIM__MSG__DETAIL__CURRENT_POSE__BUILDER_HPP_
