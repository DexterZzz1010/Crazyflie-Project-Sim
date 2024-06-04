#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy import executors

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf_transformations

from geometry_msgs.msg import Pose

class CFMove(Node):

    def __init__(self, name: str, init_dest_frame="cf_1"):
        super().__init__('move_'+name)

        self.name = name
        node_name = self.get_name()
        self.get_logger().info(f"Successfully Create: {node_name}")
        prefix =  self.name
        self.set_destination_frame(new_dest_frame=init_dest_frame)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_ = self.create_publisher(Pose, f'{prefix}/mpc_goto', 10)
        self.subscriber = self.create_subscription(
            Pose,
            f'/{prefix}/relative_pose',
            self.move_callback,
            QoSProfile(depth=1))

    def set_destination_frame(self, new_dest_frame):
        self.objective_frame = new_dest_frame

    def move_callback(self, relative_pose):
        world_pose = self.calculate_world_pose(relative_pose)
        if world_pose:
            self.publisher_.publish(world_pose)
            self.get_logger().info(f"Published Pose in world frame: {world_pose}")

    def calculate_world_pose(self, relative_pose):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'world',
                self.objective_frame,
                now)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform world to {self.objective_frame}: {ex}')
            return None

        # 获取平移向量
        translation = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        
        # 获取旋转四元数并转换为欧拉角
        rotation_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        rotation_euler = tf_transformations.euler_from_quaternion(rotation_quat)
        
        # 构建齐次坐标变换矩阵
        transform_matrix = construct_homogeneous_matrix(translation, rotation_euler)
        
        # 转换坐标
        transformed_position = transform_coordinates_homogeneous(relative_pose, transform_matrix)

        # 构造并返回世界坐标系中的Pose
        world_pose = Pose()
        world_pose.position.x, world_pose.position.y, world_pose.position.z = transformed_position
        world_pose.orientation.x = trans.transform.rotation.x
        world_pose.orientation.y = trans.transform.rotation.y
        world_pose.orientation.z = trans.transform.rotation.z
        world_pose.orientation.w = trans.transform.rotation.w

        return world_pose

def quaternion_to_euler(quat):
    explicit_quat = [quat[0], quat[1], quat[2], quat[3]]
    euler = tf_transformations.euler_from_quaternion(explicit_quat)
    return euler

def construct_homogeneous_matrix(translation, rotation_euler):
    # 创建旋转矩阵
    rotation_matrix = tf_transformations.euler_matrix(rotation_euler[0], rotation_euler[1], rotation_euler[2])[:3, :3]
    
    # 创建齐次变换矩阵
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = translation
    
    return homogeneous_matrix

def transform_coordinates_homogeneous(relative_pose, transform_matrix):
    # 将相对坐标转换为齐次坐标
    relative_position_homogeneous = np.array([relative_pose.position.x, relative_pose.position.y, relative_pose.position.z, 1.0])
    
    # 应用变换矩阵
    transformed_position_homogeneous = np.dot(transform_matrix, relative_position_homogeneous)
    
    return transformed_position_homogeneous[:3]

def main(args=None):
    N_AGENTS = 4
    rclpy.init()
    nodes = [CFMove('cf_'+str(i)) for i in np.arange(2, 1 + N_AGENTS)]
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

    rclpy.init(args=args)
    move_obj = CFMove()
    rclpy.spin(move_obj)
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
