import sys
import logging 
import numpy as np
import rowan

from crazyflie_py import *
import rclpy
import rclpy.node

from .quadrotor_simplified_model import QuadrotorSimplified
from .trajectory_tracking_mpc import TrajectoryTrackingMpc
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from crazyflie_interfaces.msg import LogDataGeneric, AttitudeSetpoint 


import pathlib

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

from time import time
from rclpy import executors
from rclpy.qos import qos_profile_sensor_data

import argparse

from ament_index_python.packages import get_package_share_directory

import tf_transformations

from enum import Enum
from copy import copy
import time
from collections import deque
from threading import Thread

class Motors(Enum):
    MOTOR_CLASSIC = 1 # https://store.bitcraze.io/products/4-x-7-mm-dc-motor-pack-for-crazyflie-2 w/ standard props
    MOTOR_UPGRADE = 2 # https://store.bitcraze.io/collections/bundles/products/thrust-upgrade-bundle-for-crazyflie-2-x

class CrazyflieMPC(rclpy.node.Node):
    def __init__(self, node_name: str, mpc_solver: TrajectoryTrackingMpc, quadrotor_dynamics: QuadrotorSimplified, mpc_N: int, mpc_tf: float, rate: int, plot_trajectory: bool = False):
        super().__init__(node_name)
        # super(CrazyflieMPC, self).__init__(target=self._start_agent)
        # self._swarm = swarm
        # self.time_helper = self._swarm.timeHelper
        # self.cfserver = self._swarm.allcfs
        # self._cf = self.cfserver.crazyfliesByName[name]
        name = self.get_name()
        prefix = '/' + name
        
        self.is_connected = True

        self.rate = rate

        self.odometry = Odometry()

        self.mpc_N = mpc_N
        self.mpc_tf = mpc_tf

        self.position = []
        self.velocity = []
        self.attitude = []

        self.path = Path()
        self.history_traj_path = Path()
        self.history_traj_path.header.frame_id = 'world'
        

        self.trajectory_changed = True

        self.flight_mode = 'idle'
        self.trajectory_t0 = self.get_clock().now()
        # self.trajectory_type = 'custom_helix'
        self.trajectory_type = 'complex'
        
        # self.trajectory_type = 'lemniscate'
        
        # TODO: Switch to parameters yaml?
        self.motors = Motors.MOTOR_CLASSIC # MOTOR_CLASSIC, MOTOR_UPGRADE

        self.takeoff_duration = 5.0
        self.land_duration = 5.0
        self.goto_duration = 5.0

        self.g = quadrotor_dynamics.gravity
        self.m = quadrotor_dynamics.mass

        self.mpc_solver = copy(mpc_solver)
        self.plot_trajectory = plot_trajectory
        self.control_queue = None
        self.get_logger().info('Initialization completed...')

        self.is_flying = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.objective_frame= "cf_1"
        
        self.create_subscription(
            PoseStamped,
            f'{prefix}/pose',
            self._pose_msg_callback,
            10)
        
        self.create_subscription(
            LogDataGeneric,
            f'{prefix}/velocity',
            self._velocity_msg_callback,
            10)
        
        self.subscriber = self.create_subscription(
            Pose,
            f'{prefix}/relative_pose',
            self._relative_pose_callback,
            10)
        
        self.mpc_solution_path_pub = self.create_publisher(
            Path,
            f'{prefix}/mpc_solution_path',
            10)
        
        self.mpc_history_path_pub = self.create_publisher(
            Path,
            f'{prefix}/history_traj_path',
            10)
        
        self.attitude_setpoint_pub = self.create_publisher(
            AttitudeSetpoint,
            f'{prefix}/cmd_attitude_setpoint',
            10)
        
        self.takeoffService = self.create_subscription(Empty, f'/all/mpc_takeoff', self.takeoff, 10)
        self.landService = self.create_subscription(Empty, f'/all/mpc_land', self.land, 10)
        self.trajectoryService = self.create_subscription(Empty, f'/all/mpc_trajectory', self.start_trajectory, 10)
        self.reletiveService = self.create_subscription(Empty, f'{prefix}/mpc_relative', self.relative_trajectory, 10)
        self.hoverService = self.create_subscription(Empty, f'/all/mpc_hover', self.hover, 10)
        self.gotoService = self.create_subscription(Pose, f'{prefix}/mpc_goto', self.goto, 10)
        self.squareService = self.create_subscription(Empty, f'/all/mpc_square', self.square, 10)


        self.create_timer(1./rate, self._main_loop)
        self.create_timer(1./10, self._mpc_solver_loop)

    def _pose_msg_callback(self, msg: PoseStamped):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.attitude = tf_transformations.euler_from_quaternion([msg.pose.orientation.x,
                                                                  msg.pose.orientation.y,
                                                                  msg.pose.orientation.z,
                                                                  msg.pose.orientation.w])
        # print(f'attitude: {np.degrees(self.attitude[2])}')
        if self.attitude[2] > np.pi:
            self.attitude[2] -= 2*np.pi
        elif self.attitude[2] < -np.pi:
            self.attitude[2] += 2*np.pi

    def _velocity_msg_callback(self, msg: LogDataGeneric):
        self.velocity = msg.values

    def _relative_pose_callback(self, msg: Pose):
        self.relative_position = [msg.position.x, msg.position.y, msg.position.z]

    def construct_homogeneous_matrix(self, translation, rotation_euler):
        # 创建旋转矩阵
        rotation_matrix = tf_transformations.euler_matrix(rotation_euler[0], rotation_euler[1], rotation_euler[2])[:3, :3]

        # 创建齐次变换矩阵
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = rotation_matrix
        homogeneous_matrix[:3, 3] = translation

        return homogeneous_matrix

    def transform_coordinates_homogeneous(self, pxr,pyr,pzr, transform_matrix):
        # 将相对坐标转换为齐次坐标
        relative_position_homogeneous = np.array([pxr,pyr,pzr, 1.0])

        # 应用变换矩阵
        transformed_position_homogeneous = np.dot(transform_matrix, relative_position_homogeneous)

        return transformed_position_homogeneous[:3]

    def set_destination_frame(self, new_dest_frame):
        self.objective_frame = new_dest_frame

    def trans_trajectory(self, pxr,pyr,pzr,vxr,vyr,vzr):
        transformed_position,transformed_velocity = self.calculate_world_pose(pxr,pyr,pzr,vxr,vyr,vzr)
        return transformed_position,transformed_velocity


    # [pxr,pyr,pzr,vxr,vyr,vzr,0.,0.,0.]
    def calculate_world_velocity(self, vxr,vyr,vzr, rotation_matrix):
        # 将相对速度转换为齐次坐标
        relative_velocity_vector = np.array([vxr,vyr,vzr])

        # 应用旋转矩阵
        transformed_velocity = np.dot(rotation_matrix, relative_velocity_vector)

        return transformed_velocity

    def calculate_world_pose(self, pxr,pyr,pzr,vxr,vyr,vzr):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('world', self.objective_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform world to {self.objective_frame}: {ex}')
            return None, None

        # 获取平移向量
        translation = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])

        # 获取旋转四元数并转换为欧拉角
        rotation_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        rotation_euler = tf_transformations.euler_from_quaternion(rotation_quat)

        # 构建齐次坐标变换矩阵
        transform_matrix = self.construct_homogeneous_matrix(translation, rotation_euler)

        # 转换位置坐标
        transformed_position = self.transform_coordinates_homogeneous(pxr,pyr,pzr,transform_matrix)
        
        rotation_matrix = transform_matrix[:3, :3]
        transformed_velocity =self.calculate_world_velocity(vxr,vyr,vzr, rotation_matrix)

        return transformed_position,transformed_velocity


    def start_trajectory(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'trajectory'

    def relative_trajectory(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'relative'

    def square(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'square'

    def takeoff(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'takeoff'
        self.go_to_position = np.array([self.position[0],
                                        self.position[1],
                                        1.0])
        
    def hover(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'hover'
        self.go_to_position = np.array([self.position[0],
                                        self.position[1],
                                        self.position[2]])
        
    def goto(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'goto'
        self.go_to_position = np.array([msg.position.x ,
                                        msg.position.y ,
                                        msg.position.z ])

    def land(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'land'
        self.go_to_position = np.array([self.position[0],
                                        self.position[1],
                                        0.1])
    def relative_function(self, t): 
        a = 1.0
        omega = 0.75*np.tanh(0.1*t)
        pxr = self.trajectory_start_position[0] + a*np.sin(-omega*t + np.pi)
        pyr = self.trajectory_start_position[1]
        pzr = self.trajectory_start_position[2] + a*np.cos(-omega*t + np.pi) + a
        vxr = -a*omega*np.cos(-omega*t + np.pi)
        vyr = 0.0
        vzr = a*omega*np.sin(-omega*t + np.pi) 
        transformed_position,transformed_velocity=self.trans_trajectory(pxr,pyr,pzr,vxr,vyr,vzr)
        return np.array([transformed_position[0],transformed_position[1],transformed_position[2],
                         transformed_velocity[0],transformed_velocity[1],transformed_velocity[1],0.,0.,0.])


    def trajectory_function(self, t):
        if self.trajectory_type == 'horizontal_circle':      
            a = 1.0
            omega = 0.75*np.tanh(0.1*t)
            pxr = self.trajectory_start_position[0] + a*np.cos(omega*t) - a
            pyr = self.trajectory_start_position[1] + a*np.sin(omega*t)
            pzr = self.trajectory_start_position[2]
            vxr = -a*omega*np.sin(omega*t)
            vyr = a*omega*np.cos(omega*t)
            vzr = 0.0
        elif self.trajectory_type == 'vertical_circle':
            a = 1.0
            omega = 0.75*np.tanh(0.1*t)
            pxr = self.trajectory_start_position[0] + a*np.sin(-omega*t + np.pi)
            pyr = self.trajectory_start_position[1]
            pzr = self.trajectory_start_position[2] + a*np.cos(-omega*t + np.pi) + a
            vxr = -a*omega*np.cos(-omega*t + np.pi)
            vyr = 0.0
            vzr = a*omega*np.sin(-omega*t + np.pi)
        elif self.trajectory_type == 'tilted_circle':
            a = 0.5
            c = 0.3
            omega = 0.75*np.tanh(0.1*t)
            pxr = self.trajectory_start_position[0] + a*np.cos(omega*t) - a
            pyr = self.trajectory_start_position[1] + a*np.sin(omega*t)
            pzr = self.trajectory_start_position[2] + c*np.sin(omega*t)
            vxr = -a*omega*np.sin(omega*t)
            vyr = a*omega*np.cos(omega*t)
            vzr = c*omega*np.cos(omega*t)
        elif self.trajectory_type == 'lemniscate':
            a = 1.0
            b = 0.5*np.tanh(0.1*t)
            pxr = self.trajectory_start_position[0] + a*np.sin(b*t)
            pyr = self.trajectory_start_position[1] + a*np.sin(b*t)*np.cos(b*t)
            pzr = self.trajectory_start_position[2]
            vxr = a*b*np.cos(b*t)
            vyr = a*b*np.cos(2*b*t)
            vzr = 0.0
        elif self.trajectory_type == 'helix':
            a = 1.0
            T_end = 10.0
            helix_velocity = 0.2
            omega = 0.75*np.tanh(0.1*t)
            pxr = self.trajectory_start_position[0] + a*np.cos(omega*t) - a
            pyr = self.trajectory_start_position[1] + a*np.sin(omega*t)
            vxr = -a*omega*np.sin(omega*t)
            vyr = a*omega*np.cos(omega*t)
            if t < T_end:
                pzr = self.trajectory_start_position[2] + helix_velocity*t
                vzr = helix_velocity
            else:
                pzr = self.trajectory_start_position[2] + helix_velocity*T_end
                vzr = 0.0
        
        elif self.trajectory_type == 'custom_helix':
            T_end = 10.0
            helix_velocity = 0.2

            # Define the center of the helix to be (0.5, 0.5, z)
            center_x = 1
            center_y = 1
            center_z = self.trajectory_start_position[2]

            # Calculate the radius based on the initial position
            a = np.sqrt((self.trajectory_start_position[0] - center_x) ** 2 +
                        (self.trajectory_start_position[1] - center_y) ** 2)

            # Calculate the initial phase angle
            initial_phase = np.arctan2(self.trajectory_start_position[1] - center_y,
                                    self.trajectory_start_position[0] - center_x)

            # Define omega as a function of time
            omega = 0.5 * np.tanh(0.1 * t)

            # Compute the helix trajectory relative to the new center
            pxr = center_x + a * np.cos(omega * t + initial_phase)
            pyr = center_y + a * np.sin(omega * t + initial_phase)
            vxr = -a * omega * np.sin(omega * t + initial_phase)
            vyr = a * omega * np.cos(omega * t + initial_phase)

            # Update the z-coordinate with helix motion
            if t < T_end:
                pzr = center_z + helix_velocity * t
                vzr = helix_velocity/2
            else:
                pzr = center_z + helix_velocity * T_end
                vzr = 0.0


        elif self.trajectory_type == 'spiral':
            a = 1.0  # 半径变化率
            omega = 0.75  # 角速度
            vertical_velocity = 0.1  # 沿z轴的速度
            cx, cy, cz = [0,0,0]  # 旋转中心
            omega = 0.75*np.tanh(0.1*t)
            pxr = cx+ a*np.cos(omega*t) - a
            pyr = cy+ a*np.sin(omega*t)
            vxr = -a*omega*np.sin(omega*t)
            vyr = a*omega*np.cos(omega*t)
            # vzr = vertical_velocity
            if t < T_end:
                pzr = self.trajectory_start_position[2] + vertical_velocity*t
                vzr = vertical_velocity
            else:
                pzr = self.trajectory_start_position[2] + vertical_velocity*T_end
                vzr = 0.0
        
        elif self.trajectory_type == 'square':
            a = 1.0  # 边长
            T_edge = 5.0  # 每条边所需时间
            t_mod = t % (4 * T_edge)  # 将时间映射到一个周期内
            
            if t_mod < T_edge:  # 第一条边
                pxr = self.trajectory_start_position[0] + a * (t_mod / T_edge)
                pyr = self.trajectory_start_position[1]
                pzr = self.trajectory_start_position[2]
                vxr = a / T_edge
                vyr = 0.0
                vzr = 0.0
            elif t_mod < 2 * T_edge:  # 第二条边
                pxr = self.trajectory_start_position[0] + a
                pyr = self.trajectory_start_position[1] + a * ((t_mod - T_edge) / T_edge)
                pzr = self.trajectory_start_position[2]
                vxr = 0.0
                vyr = a / T_edge
                vzr = 0.0
            elif t_mod < 3 * T_edge:  # 第三条边
                pxr = self.trajectory_start_position[0] + a * (1 - (t_mod - 2 * T_edge) / T_edge)
                pyr = self.trajectory_start_position[1] + a
                pzr = self.trajectory_start_position[2]
                vxr = -a / T_edge
                vyr = 0.0
                vzr = 0.0
            else:  # 第四条边
                pxr = self.trajectory_start_position[0]
                pyr = self.trajectory_start_position[1] + a * (1 - (t_mod - 3 * T_edge) / T_edge)
                pzr = self.trajectory_start_position[2]
                vxr = 0.0
                vyr = -a / T_edge
                vzr = 0.0
        elif self.trajectory_type == 'complex':
            a = 0.15
            b = 0.1
            delta = np.pi / 2
            omega1 = 0.5
            omega2 = 0.75

            # Generate Lissajous curve for closed-loop trajectory
            pxr = self.trajectory_start_position[0] + a * np.sin(omega1 * t + delta)
            pyr = self.trajectory_start_position[1] + b * np.sin(omega2 * t)
            pzr = self.trajectory_start_position[2] + a * np.sin(omega1 * t)

            vxr = a * omega1 * np.cos(omega1 * t + delta)
            vyr = b * omega2 * np.cos(omega2 * t)
            vzr = a * omega1 * np.cos(omega1 * t)

        return np.array([pxr,pyr,pzr,vxr,vyr,vzr,0.,0.,0.])
    
    def square_function(self, t):
        a = 1.0  # 边长
        T_edge = 5.0  # 每条边所需时间
        t_mod = t % (4 * T_edge)  # 将时间映射到一个周期内
        
        if t_mod < T_edge:  # 第一条边
            pxr = self.trajectory_start_position[0] + a * (t_mod / T_edge)
            pyr = self.trajectory_start_position[1]
            pzr = self.trajectory_start_position[2]
            vxr = a / T_edge
            vyr = 0.0
            vzr = 0.0
        elif t_mod < 2 * T_edge:  # 第二条边
            pxr = self.trajectory_start_position[0] + a
            pyr = self.trajectory_start_position[1] + a * ((t_mod - T_edge) / T_edge)
            pzr = self.trajectory_start_position[2]
            vxr = 0.0
            vyr = a / T_edge
            vzr = 0.0
        elif t_mod < 3 * T_edge:  # 第三条边
            pxr = self.trajectory_start_position[0] + a * (1 - (t_mod - 2 * T_edge) / T_edge)
            pyr = self.trajectory_start_position[1] + a
            pzr = self.trajectory_start_position[2]
            vxr = -a / T_edge
            vyr = 0.0
            vzr = 0.0
        else:  # 第四条边
            pxr = self.trajectory_start_position[0]
            pyr = self.trajectory_start_position[1] + a * (1 - (t_mod - 3 * T_edge) / T_edge)
            pzr = self.trajectory_start_position[2]
            vxr = 0.0
            vyr = -a / T_edge
            vzr = 0.0
        return np.array([pxr,pyr,pzr,vxr,vyr,vzr,0.,0.,0.])

    def navigator(self, t):
        if self.flight_mode == 'takeoff':
            t_mpc_array = np.linspace(t, self.mpc_tf + t, self.mpc_N+1)
            yref = np.array([np.array([*((self.go_to_position - self.trajectory_start_position)*(1./(1. + np.exp(-(12.0 * (t_mpc - self.takeoff_duration) / self.takeoff_duration + 6.0)))) + self.trajectory_start_position),0.,0.,0.,0.,0.,0.]) for t_mpc in t_mpc_array]).T
            # yref = np.repeat(np.array([[*self.go_to_position,0,0,0]]).T, self.mpc_N, axis=1)
        elif self.flight_mode == 'land':
            t_mpc_array = np.linspace(t, self.mpc_tf + t, self.mpc_N+1)
            yref = np.array([np.array([*((self.go_to_position - self.trajectory_start_position)*(1./(1. + np.exp(-(12.0 * (t_mpc - self.land_duration) / self.land_duration + 6.0)))) + self.trajectory_start_position),0.,0.,0.,0.,0.,0.]) for t_mpc in t_mpc_array]).T
            # yref = np.repeat(np.array([[*self.go_to_position,0,0,0]]).T, self.mpc_N, axis=1)
        elif self.flight_mode == 'trajectory':
            t_mpc_array = np.linspace(t, self.mpc_tf + t, self.mpc_N+1)
            yref = np.array([self.trajectory_function(t_mpc) for t_mpc in t_mpc_array]).T
        elif self.flight_mode == 'relative':
            t_mpc_array = np.linspace(t, self.mpc_tf + t, self.mpc_N+1)
            yref = np.array([self.relative_function(t_mpc) for t_mpc in t_mpc_array]).T
        elif self.flight_mode == 'square':
            t_mpc_array = np.linspace(t, self.mpc_tf + t, self.mpc_N+1)
            yref = np.array([self.square_function(t_mpc) for t_mpc in t_mpc_array]).T
        elif self.flight_mode == 'hover':
            t_mpc_array = np.linspace(t, self.mpc_tf + t, self.mpc_N+1)
            yref = np.array([np.array([*((self.go_to_position - self.trajectory_start_position)*(1./(1. + np.exp(-(12.0 * (t_mpc - self.land_duration) / self.land_duration + 6.0)))) + self.trajectory_start_position),0.,0.,0.,0.,0.,0.]) for t_mpc in t_mpc_array]).T
            # yref = np.repeat(np.array([[*self.go_to_position,0.,0.,0.,0.,0.,0.]]).T, self.mpc_N, axis=1)
        elif self.flight_mode == 'goto':
            t_mpc_array = np.linspace(t, self.mpc_tf + t, self.mpc_N+1)
            yref = np.array([np.array([*((self.go_to_position - self.trajectory_start_position)*(1./(1. + np.exp(-(12.0 * (t_mpc - self.goto_duration) / self.goto_duration  + 6.0)))) + self.trajectory_start_position),0.,0.,0.,0.,0.,0.]) for t_mpc in t_mpc_array]).T
        
        return yref
    
    def cmd_attitude_setpoint(self, roll, pitch, yaw_rate, thrust_pwm):
        setpoint = AttitudeSetpoint()
        setpoint.roll = roll
        setpoint.pitch = pitch
        setpoint.yaw_rate = yaw_rate
        setpoint.thrust = thrust_pwm
        self.attitude_setpoint_pub.publish(setpoint)

    def thrust_to_pwm(self, collective_thrust: float) -> int:
        # omega_per_rotor = 7460.8*np.sqrt((collective_thrust / 4.0))
        # pwm_per_rotor = 24.5307*(omega_per_rotor - 380.8359)
        collective_thrust = max(collective_thrust, 0.) #  make sure it's not negative
        if self.motors == Motors.MOTOR_CLASSIC:
            return int(max(min(24.5307*(7460.8*np.sqrt((collective_thrust / 4.0)) - 380.8359), 65535),0))
        elif self.motors == Motors.MOTOR_UPGRADE:
            return int(max(min(24.5307*(6462.1*np.sqrt((collective_thrust / 4.0)) - 380.8359), 65535),0))
    
    def plot_history_trajectory(self):
        history_pose = PoseStamped()
        history_pose.header.stamp = self.get_clock().now().to_msg()
        history_pose.pose.position.x = self.position[0]
        history_pose.pose.position.y = self.position[1]
        history_pose.pose.position.z = self.position[2]
        # 将当前位姿追加到历史轨迹中
        self.history_traj_path.poses.append(history_pose)
        # 更新头信息的时间戳
        self.history_traj_path.header.stamp = self.get_clock().now().to_msg()
        # 发布历史轨迹
        self.mpc_history_path_pub.publish(self.history_traj_path)

    def _mpc_solver_loop(self):
        if not self.is_flying:
            return
        
        if self.trajectory_changed:
            self.trajectory_start_position = self.position
            self.trajectory_t0 = self.get_clock().now()
            self.trajectory_changed = False

        t = (self.get_clock().now() - self.trajectory_t0).nanoseconds / 10.0**9

        x0 = np.array([
            *self.position,
            *self.velocity,
            *self.attitude
        ])

        trajectory = self.navigator(t)
        yref = trajectory[:,:-1]
        yref_e = trajectory[:,-1]
        
        status, x_mpc, u_mpc = self.mpc_solver.solve_mpc(x0, yref, yref_e)
        self.control_queue = deque(u_mpc)

        if self.plot_trajectory:
            mpc_solution_path = Path()
            mpc_solution_path.header.frame_id = 'world'
            mpc_solution_path.header.stamp = self.get_clock().now().to_msg()

            for i in range(self.mpc_N):
                mpc_pose = PoseStamped()
                mpc_pose.pose.position.x = x_mpc[i,0]
                mpc_pose.pose.position.y = x_mpc[i,1]
                mpc_pose.pose.position.z = x_mpc[i,2]
                mpc_solution_path.poses.append(mpc_pose)

            self.mpc_solution_path_pub.publish(mpc_solution_path)
            self.plot_history_trajectory()

    def _main_loop(self):
        if self.flight_mode == 'idle':
            return

        if not self.position or not self.velocity or not self.attitude:
            self.get_logger().warning("Empty state message.")
            return
        
        if not self.is_flying:
            self.is_flying = True
            self.cmd_attitude_setpoint(0.,0.,0.,0)

        if self.control_queue is not None:
            control = self.control_queue.popleft()
            thrust_pwm = self.thrust_to_pwm(control[3])
            yawrate = 3.*(np.degrees(self.attitude[2]))
            self.cmd_attitude_setpoint(np.degrees(control[0]), 
                                    np.degrees(control[1]), 
                                    yawrate, 
                                    thrust_pwm)
        

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-n","--n_agents",help ="Number of agents",default=1,type=int)
    parser.add_argument("--build_acados", help="Build acados", action='store_true')
    args = parser.parse_args()
    N_AGENTS = args.n_agents
    build_acados = args.build_acados

    rclpy.init()

    # Quadrotor Parameters
    mass = 0.028
    arm_length=0.044
    Ixx=2.3951e-5
    Iyy=2.3951e-5
    Izz=3.2347e-5
    cm=2.4e-6
    tau=0.08

    # MPC Parameters
    mpc_tf = 1.0
    mpc_N = 50
    control_update_rate = 50
    plot_trajectory = True

    quadrotor_dynamics = QuadrotorSimplified(mass, arm_length, Ixx, Iyy, Izz, cm, tau)
    acados_c_generated_code_path = pathlib.Path(get_package_share_directory('crazyflie_mpc')).resolve() / 'acados_generated_files'
    mpc_solver = TrajectoryTrackingMpc('crazyflie', quadrotor_dynamics, mpc_tf, mpc_N, code_export_directory=acados_c_generated_code_path)
    if build_acados:
        mpc_solver.generate_mpc()
    # 创建对象 循环
    nodes = [CrazyflieMPC('cf_'+str(i), mpc_solver, quadrotor_dynamics, mpc_N, mpc_tf, control_update_rate, plot_trajectory) for i in np.arange(1, 1 + N_AGENTS)]
    executor = executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        while rclpy.ok():
            node.get_logger().info('Beginning multiagent executor, shut down with CTRL-C')
            executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
