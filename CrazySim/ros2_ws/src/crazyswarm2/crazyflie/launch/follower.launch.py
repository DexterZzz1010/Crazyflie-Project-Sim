from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crazyflie',  # 替换为你的包名
            executable='cf_tf_follower',    # 替换为你的跟随节点可执行文件名
            name='cf_follower_node',
            output='screen'
        ),
        Node(
            package='crazyflie',  # 替换为你的包名
            executable='cf2_listener',  # 替换为你的控制器节点可执行文件名
            name='cf2_listener_node',
            output='screen'
        )
    ])
