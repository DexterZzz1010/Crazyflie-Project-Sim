#!/bin/bash

# 定义工作目录和ROS2工作空间的source命令
ROS_DIR="/home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws"
SIM_DIR="/home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/crazyflie-firmware"
ROS_SETUP="source install/setup.bash;"

# 打开第一个终端并发布消息到 mpc takeoff
gnome-terminal -- bash -c "
cd $ROS_DIR;
$ROS_SETUP
ros2 topic pub -t 1 /all/mpc_takeoff std_msgs/msg/Empty;
exec bash"

sleep 6

# 打开第一个终端并发布消息到 cf2
gnome-terminal -- bash -c "
cd $ROS_DIR;
$ROS_SETUP
ros2 topic pub -t 1 /cf_2/relative_pose geometry_msgs/Pose '{position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}';
exec bash"

# 打开第二个终端并发布消息到 cf3
gnome-terminal -- bash -c "
cd $ROS_DIR;
$ROS_SETUP
ros2 topic pub -t 1 /cf_3/relative_pose geometry_msgs/Pose '{position: {x: 0.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}';
exec bash"

# 打开第三个终端并发布消息到 cf4
gnome-terminal -- bash -c "
cd $WORK_DIR;
$ROS_SETUP
ros2 topic pub -t 1 /cf_4/relative_pose geometry_msgs/Pose '{position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}';
exec bash"
