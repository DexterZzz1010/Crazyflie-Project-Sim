// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crazyflie_interfaces:msg/PoseVelocity.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crazyflie_interfaces/msg/detail/pose_velocity__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace crazyflie_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PoseVelocity_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crazyflie_interfaces::msg::PoseVelocity(_init);
}

void PoseVelocity_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crazyflie_interfaces::msg::PoseVelocity *>(message_memory);
  typed_message->~PoseVelocity();
}

size_t size_function__PoseVelocity__position(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__PoseVelocity__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__PoseVelocity__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__PoseVelocity__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PoseVelocity__position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PoseVelocity__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PoseVelocity__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__PoseVelocity__velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__PoseVelocity__velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__PoseVelocity__velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__PoseVelocity__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PoseVelocity__velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PoseVelocity__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PoseVelocity__velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PoseVelocity_message_member_array[2] = {
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::PoseVelocity, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__PoseVelocity__position,  // size() function pointer
    get_const_function__PoseVelocity__position,  // get_const(index) function pointer
    get_function__PoseVelocity__position,  // get(index) function pointer
    fetch_function__PoseVelocity__position,  // fetch(index, &value) function pointer
    assign_function__PoseVelocity__position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::PoseVelocity, velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__PoseVelocity__velocity,  // size() function pointer
    get_const_function__PoseVelocity__velocity,  // get_const(index) function pointer
    get_function__PoseVelocity__velocity,  // get(index) function pointer
    fetch_function__PoseVelocity__velocity,  // fetch(index, &value) function pointer
    assign_function__PoseVelocity__velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PoseVelocity_message_members = {
  "crazyflie_interfaces::msg",  // message namespace
  "PoseVelocity",  // message name
  2,  // number of fields
  sizeof(crazyflie_interfaces::msg::PoseVelocity),
  PoseVelocity_message_member_array,  // message members
  PoseVelocity_init_function,  // function to initialize message memory (memory has to be allocated)
  PoseVelocity_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PoseVelocity_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PoseVelocity_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace crazyflie_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<crazyflie_interfaces::msg::PoseVelocity>()
{
  return &::crazyflie_interfaces::msg::rosidl_typesupport_introspection_cpp::PoseVelocity_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crazyflie_interfaces, msg, PoseVelocity)() {
  return &::crazyflie_interfaces::msg::rosidl_typesupport_introspection_cpp::PoseVelocity_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
