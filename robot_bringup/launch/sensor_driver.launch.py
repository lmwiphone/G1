#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():


    LDS_LAUNCH_FILE = '/msg_MID360_launch.py'
    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2'))
    start_livox = LaunchConfiguration('start_livox')

    return LaunchDescription([

        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('livox_config', default_value=os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'config',
            'G1_MID360s_config.json')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            condition=IfCondition(start_livox),
            launch_arguments={'livox_config': LaunchConfiguration('livox_config')}.items(),
        ),
        Node(
            package='lightning',
            executable='livox_custom_to_pointcloud2',
            name='livox_custom_to_pointcloud2',
            parameters=[{
                'input_topic': '/livox/lidar',
                'output_topic': '/livox/points',
                'frame_id': 'mid360_link',
                'point_stride': 1,
            }],
            output='screen',
        )
    ])
