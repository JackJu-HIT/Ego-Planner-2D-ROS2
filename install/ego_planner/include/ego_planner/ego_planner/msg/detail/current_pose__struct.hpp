// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ego_planner:msg/CurrentPose.idl
// generated code does not contain a copyright notice

#ifndef EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__STRUCT_HPP_
#define EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ego_planner__msg__CurrentPose __attribute__((deprecated))
#else
# define DEPRECATED__ego_planner__msg__CurrentPose __declspec(deprecated)
#endif

namespace ego_planner
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CurrentPose_
{
  using Type = CurrentPose_<ContainerAllocator>;

  explicit CurrentPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  explicit CurrentPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _theta_type =
    double;
  _theta_type theta;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ego_planner::msg::CurrentPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const ego_planner::msg::CurrentPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ego_planner::msg::CurrentPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ego_planner::msg::CurrentPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ego_planner::msg::CurrentPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ego_planner::msg::CurrentPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ego_planner::msg::CurrentPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ego_planner::msg::CurrentPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ego_planner::msg::CurrentPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ego_planner::msg::CurrentPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ego_planner__msg__CurrentPose
    std::shared_ptr<ego_planner::msg::CurrentPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ego_planner__msg__CurrentPose
    std::shared_ptr<ego_planner::msg::CurrentPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CurrentPose_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    return true;
  }
  bool operator!=(const CurrentPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CurrentPose_

// alias to use template instance with default allocator
using CurrentPose =
  ego_planner::msg::CurrentPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ego_planner

#endif  // EGO_PLANNER__MSG__DETAIL__CURRENT_POSE__STRUCT_HPP_
