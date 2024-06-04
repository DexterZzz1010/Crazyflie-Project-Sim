#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf_transformations

from std_msgs.msg import String
from geometry_msgs.msg import Pose

class Coordinates:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class CFMove(Node):

    def __init__(self, timer_period=0.05, model_name="cf_2", init_dest_frame="cf_1", TAKEOFF_DURATION=2.5, HOVER_DURATION=5.0):
        super().__init__('force_move_crazyflies')

        self._model_name = model_name
        self.set_destination_frame(new_dest_frame=init_dest_frame)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer_period = timer_period
        self.timer_rate = 1.0 / self.timer_period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.publisher_ = self.create_publisher(Pose, 'cf_2/mpc_goto', 10)

        self.subscriber = self.create_subscription(
            Pose,
            '/cf_2/relative_pose',
            self.move_callback,
            QoSProfile(depth=1))

    def timer_callback(self):
        pass  # 定时器回调不再需要执行任何操作

    def set_destination_frame(self, new_dest_frame):
        self.objective_frame = new_dest_frame

    def move_callback(self, msg):
        relative_pose = msg
        world_pose = self.calculate_world_pose(relative_pose)
        if world_pose:
            self.publisher_.publish(world_pose)

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

        # Apply the relative pose to the transform from world to cf_1
        world_pose = Pose()
        world_pose.position.x = trans.transform.translation.x + relative_pose.position.x
        world_pose.position.y = trans.transform.translation.y + relative_pose.position.y
        world_pose.position.z = trans.transform.translation.z + relative_pose.position.z

        quaternion = tf_transformations.quaternion_multiply(
            [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w],
            [relative_pose.orientation.x, relative_pose.orientation.y, relative_pose.orientation.z, relative_pose.orientation.w]
        )
        world_pose.orientation.x = quaternion[0]
        world_pose.orientation.y = quaternion[1]
        world_pose.orientation.z = quaternion[2]
        world_pose.orientation.w = quaternion[3]

        return world_pose

def main(args=None):
    rclpy.init(args=args)
    move_obj = CFMove()
    rclpy.spin(move_obj)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
