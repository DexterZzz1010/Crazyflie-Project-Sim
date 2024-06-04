// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/PoseVelocity.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__POSE_VELOCITY__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__POSE_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/pose_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_PoseVelocity_velocity
{
public:
  explicit Init_PoseVelocity_velocity(::crazyflie_interfaces::msg::PoseVelocity & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::PoseVelocity velocity(::crazyflie_interfaces::msg::PoseVelocity::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::PoseVelocity msg_;
};

class Init_PoseVelocity_position
{
public:
  Init_PoseVelocity_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseVelocity_velocity position(::crazyflie_interfaces::msg::PoseVelocity::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_PoseVelocity_velocity(msg_);
  }

private:
  ::crazyflie_interfaces::msg::PoseVelocity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::PoseVelocity>()
{
  return crazyflie_interfaces::msg::builder::Init_PoseVelocity_position();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__POSE_VELOCITY__BUILDER_HPP_
