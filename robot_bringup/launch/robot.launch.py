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


    robot_bringup_pkg_dir = LaunchConfiguration(
        'robot_bringup_pkg_dir',
        default=os.path.join(get_package_share_directory('robot_bringup'), 'launch'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    start_livox = LaunchConfiguration('start_livox')


    return LaunchDescription([
        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('start_collision_monitor', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/rosbridge_websocket_launch.py"]),
            launch_arguments={}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/collision_monitor.launch.py']),
            condition=IfCondition(LaunchConfiguration('start_collision_monitor')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('g1_nav_bridge'),
                             'launch', 'nav_bridge.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [robot_bringup_pkg_dir, '/sensor_driver.launch.py']),
            launch_arguments={'start_livox': start_livox}.items(),
        ),

        Node(
            prefix=["taskset -c 3-7"],
            package='robot_bringup',
            executable='robot_pose_pub_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            prefix=["taskset -c 3-7"],
            package='robot_bringup',
            executable='editor_map_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            prefix=["taskset -c 3-7"],
            package='aid_robot_py',
            executable='map_manager_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            prefix=["taskset -c 3-7"],
            package='robot_bringup',
            executable='robot_status_manager_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            prefix=["taskset -c 3-7"],
            package='aid_robot_py',
            executable='map_transform_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            prefix=["taskset -c 3-7"],
            package='aid_robot_py',
            executable='launch_manager_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            prefix=["taskset -c 3-7"],
            package='robot_bringup',
            executable='forbidden_map_create_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            prefix=["taskset -c 3-7"],
            package='aid_robot_py',
            executable='waypoint_manage_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

    ])
