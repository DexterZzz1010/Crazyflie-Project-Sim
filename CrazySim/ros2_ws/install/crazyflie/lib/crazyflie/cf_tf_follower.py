#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped

class CrazyflieFollower(Node):
    def __init__(self):
        super().__init__('crazyflie_tf_follower')
        self.make_transform()
        self.subscription = self.create_subscription(
            PoseStamped,
            '/cf_1/pose',
            self.pose_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def make_transform(self):
        self.transform = TransformStamped()
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = 'world'
        self.transform.child_frame_id = 'cf_1'


    def pose_callback(self, msg):

        self.cf_pose = msg
        
        # Setting a fixed position relative to cf1
        self.transform.transform.translation.x = msg.pose.position.x
        self.transform.transform.translation.y = msg.pose.position.y
        self.transform.transform.translation.z = msg.pose.position.z
        
        # Assuming no rotation relative to cf1
        self.transform.transform.rotation = msg.pose.orientation
        self.get_logger().debug('Pose VALUE: "%s"' % str(self.cf_pose))
        self.tf_broadcaster.sendTransform(self.transform)

def main(args=None):
    rclpy.init()
    follower = CrazyflieFollower()
    rclpy.spin(follower)

if __name__ == '__main__':
    main()
