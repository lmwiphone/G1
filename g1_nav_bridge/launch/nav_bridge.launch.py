import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('g1_nav_bridge'), 'config', 'nav_bridge.yaml')
    return LaunchDescription([
        Node(
            package='g1_nav_bridge',
            executable='cmdvel_to_sport',
            name='g1_cmdvel_to_sport',
            output='screen',
            parameters=[config],
        )
    ])
