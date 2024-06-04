cd crazyflies/group-crazyflie-1/test/sim_test/CrazySim/


bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 4 -m crazyflie

ros2 launch crazyflie launch.py backend:=cflib

ros2 run crazyflie_mpc crazyflie_multiagent_mpc --n_agents=4 --build_acados

ros2 topic pub -t 1 /all/mpc_takeoff std_msgs/msg/Empty

ros2 topic pub -t 1 /all/mpc_trajectory std_msgs/msg/Empty

ros2 topic pub -t 1 /all/mpc_square std_msgs/msg/Empty

ros2 topic pub /cf_1/mpc_goto geometry_msgs/Pose "{
  position: {x: -2.0, y: -2.0, z: 1.0},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
}"

ros2 topic pub /cf_2/relative_pose geometry_msgs/Pose "{
  position: {x: 2.0, y: 2.0, z: 1.0},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
}"

ros2 topic echo /cf_2/pose

