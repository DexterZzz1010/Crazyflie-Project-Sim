// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:msg/PoseVelocity.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/msg/detail/pose_velocity__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/msg/detail/pose_velocity__functions.h"
#include "crazyflie_interfaces/msg/detail/pose_velocity__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__msg__PoseVelocity__init(message_memory);
}

void crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_fini_function(void * message_memory)
{
  crazyflie_interfaces__msg__PoseVelocity__fini(message_memory);
}

size_t crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__size_function__PoseVelocity__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_const_function__PoseVelocity__position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_function__PoseVelocity__position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__fetch_function__PoseVelocity__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_const_function__PoseVelocity__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__assign_function__PoseVelocity__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_function__PoseVelocity__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__size_function__PoseVelocity__velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_const_function__PoseVelocity__velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_function__PoseVelocity__velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__fetch_function__PoseVelocity__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_const_function__PoseVelocity__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__assign_function__PoseVelocity__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_function__PoseVelocity__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_message_member_array[2] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__PoseVelocity, position),  // bytes offset in struct
    NULL,  // default value
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__size_function__PoseVelocity__position,  // size() function pointer
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_const_function__PoseVelocity__position,  // get_const(index) function pointer
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_function__PoseVelocity__position,  // get(index) function pointer
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__fetch_function__PoseVelocity__position,  // fetch(index, &value) function pointer
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__assign_function__PoseVelocity__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__PoseVelocity, velocity),  // bytes offset in struct
    NULL,  // default value
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__size_function__PoseVelocity__velocity,  // size() function pointer
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_const_function__PoseVelocity__velocity,  // get_const(index) function pointer
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__get_function__PoseVelocity__velocity,  // get(index) function pointer
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__fetch_function__PoseVelocity__velocity,  // fetch(index, &value) function pointer
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__assign_function__PoseVelocity__velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_message_members = {
  "crazyflie_interfaces__msg",  // message namespace
  "PoseVelocity",  // message name
  2,  // number of fields
  sizeof(crazyflie_interfaces__msg__PoseVelocity),
  crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_message_member_array,  // message members
  crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_message_type_support_handle = {
  0,
  &crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, msg, PoseVelocity)() {
  if (!crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__msg__PoseVelocity__rosidl_typesupport_introspection_c__PoseVelocity_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
