#! /usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'cf_1']  # 'odom'到'world'
    )

    return LaunchDescription(
        [
            static_tf_pub # static_tf_pub 节点名
        ]
    )
