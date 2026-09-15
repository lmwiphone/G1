import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('g1_nav_bridge'), 'config', 'nav_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_safe'),
        Node(package='g1_nav_bridge', executable='tf_to_current_pose',
             name='g1_tf_to_current_pose', output='screen'),
        Node(package='g1_nav_bridge', executable='sport_to_odom',
             name='g1_sport_to_odom', output='screen', parameters=[config]),
        Node(
            package='g1_nav_bridge',
            executable='cmdvel_to_sport',
            name='g1_cmdvel_to_sport',
            output='screen',
            parameters=[config, {
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }],
        )
    ])
