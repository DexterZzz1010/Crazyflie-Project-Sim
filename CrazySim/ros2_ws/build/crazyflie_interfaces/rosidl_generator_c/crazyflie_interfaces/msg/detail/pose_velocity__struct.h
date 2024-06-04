// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/PoseVelocity.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__POSE_VELOCITY__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__POSE_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PoseVelocity in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__PoseVelocity
{
  /// [px, py, pz]
  double position[3];
  /// [vx, vy, vz]
  double velocity[3];
} crazyflie_interfaces__msg__PoseVelocity;

// Struct for a sequence of crazyflie_interfaces__msg__PoseVelocity.
typedef struct crazyflie_interfaces__msg__PoseVelocity__Sequence
{
  crazyflie_interfaces__msg__PoseVelocity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__PoseVelocity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__POSE_VELOCITY__STRUCT_H_
