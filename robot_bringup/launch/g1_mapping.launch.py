#!/usr/bin/env python3
"""G1 online mapping using Lightning-LM (sensor driver is system-owned)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lightning_launch = os.path.join(
        get_package_share_directory('lightning'), 'launch', 'g1_mapping.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('with_ui', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lightning_launch),
            launch_arguments={
                'start_livox': 'false',
                'with_ui': LaunchConfiguration('with_ui'),
                'start_rviz': LaunchConfiguration('start_rviz'),
            }.items(),
        ),
    ])
