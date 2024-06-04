#!/bin/bash

# 定义工作目录和ROS2工作空间的source命令
ROS_DIR="/home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/ros2_ws"
SIM_DIR="/home/zyf/crazyflies/group-crazyflie-1/test/sim_test/CrazySim/crazyflie-firmware"
ROS_SETUP="source install/setup.bash;"

# 打开第一个终端并打开gazebo
gnome-terminal --tab --title="gazebo" -- bash -c "
cd $SIM_DIR;
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 4 -m crazyflie"

sleep 7

# 打开第二个终端并运行rviz2
gnome-terminal --tab --title="rviz2-crazyswarm2" -- bash -c "
cd $ROS_DIR;
$ROS_SETUP
ros2 launch crazyflie launch.py backend:=cflib"

sleep 5

# 打开第三个终端并运行acados
gnome-terminal --tab --title="acados-mpc" -- bash -c "
cd $ROS_DIR;
$ROS_SETUP
ros2 run crazyflie_mpc crazyflie_multiagent_mpc --n_agents=4 --build_acados"

sleep 8

# 打开第四个终端并运行tf listener
gnome-terminal --tab --title="cf_move" -- bash -c "
cd $ROS_DIR;
$ROS_SETUP
ros2 run crazyflie multiagent_cf_move.py"
