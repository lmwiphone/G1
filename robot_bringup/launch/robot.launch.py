#!/usr/bin/env python3
"""G1 唯一顶层入口。

本文件只负责编排已有的原子 launch，不在这里重复创建 Livox、Lightning、
Nav2、运动桥或 collision_monitor 节点。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _is_mode(*modes):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('mode'), "' in ", repr(tuple(modes))
    ]))


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    bridge_share = get_package_share_directory('g1_nav_bridge')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox = LaunchConfiguration('start_livox')
    map_dir = LaunchConfiguration('map_dir')
    with_ui = LaunchConfiguration('with_ui')
    with_2dui = LaunchConfiguration('with_2dui')
    start_rviz = LaunchConfiguration('start_rviz')

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sensor_driver.launch.py')),
        launch_arguments={
            'start_livox': start_livox,
            'livox_config': LaunchConfiguration('livox_config'),
        }.items(),
    )

    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'rosbridge_websocket_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_rosbridge')),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'g1_mapping.launch.py')),
        condition=_is_mode('mapping'),
        launch_arguments={
            'map_dir': map_dir,
            'with_ui': with_ui,
            'with_2dui': with_2dui,
            'start_rviz': start_rviz,
            'floor_height': LaunchConfiguration('floor_height'),
            'min_obstacle_height': LaunchConfiguration('min_obstacle_height'),
            'max_obstacle_height': LaunchConfiguration('max_obstacle_height'),
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'g1_localization.launch.py')),
        condition=_is_mode('localization', 'navigation'),
        launch_arguments={
            'map_dir': map_dir,
            'with_ui': with_ui,
            'with_2dui': with_2dui,
            'start_rviz': start_rviz,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'g1_navigation.launch.py')),
        condition=_is_mode('navigation'),
        launch_arguments={
            'map': LaunchConfiguration('nav_map'),
            'floor_z': LaunchConfiguration('floor_z'),
            'base_floor_z': LaunchConfiguration('base_floor_z'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'sensor_frame': LaunchConfiguration('sensor_frame'),
            'robot_radius': LaunchConfiguration('robot_radius'),
            'use_collision_monitor': LaunchConfiguration('use_collision_monitor'),
            'use_keepout': LaunchConfiguration('use_keepout'),
            # 运动桥只由 g1_navigation.launch.py 创建一次。
            'start_bridge': 'true',
        }.items(),
    )

    backend_nodes = GroupAction(
        condition=IfCondition(LaunchConfiguration('start_backend')),
        actions=[
            Node(package='robot_bringup', executable='robot_pose_pub_node',
                 name='robot_pose_pub_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='robot_bringup', executable='editor_map_node',
                 name='editor_map_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='aid_robot_py', executable='map_manager_node',
                 name='map_manager_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='robot_bringup', executable='robot_status_manager_node',
                 name='robot_status_manager_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='aid_robot_py', executable='map_transform_node',
                 name='map_transform_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='aid_robot_py', executable='launch_manager_node',
                 name='launch_manager_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='robot_bringup', executable='forbidden_map_create_node',
                 name='forbidden_map_create_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='aid_robot_py', executable='waypoint_manage_node',
                 name='waypoint_manage_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
        ],
    )

    default_map_dir = '/opt/G1/lighting_ws/data/new_map'
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='base',
            choices=['base', 'mapping', 'localization', 'navigation'],
            description='互斥运行模式；navigation 会组合定位与 Nav2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('start_rosbridge', default_value='true'),
        DeclareLaunchArgument('start_backend', default_value='true'),
        DeclareLaunchArgument('with_ui', default_value='false'),
        DeclareLaunchArgument('with_2dui', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('map_dir', default_value=default_map_dir),
        DeclareLaunchArgument('nav_map', default_value=default_map_dir + '/map.yaml'),
        DeclareLaunchArgument('floor_height', default_value=''),
        DeclareLaunchArgument('min_obstacle_height', default_value=''),
        DeclareLaunchArgument('max_obstacle_height', default_value=''),
        DeclareLaunchArgument('floor_z', default_value='0.0'),
        DeclareLaunchArgument('base_floor_z', default_value='0.0'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/points'),
        DeclareLaunchArgument('sensor_frame', default_value='mid360_link'),
        DeclareLaunchArgument('robot_radius', default_value='0.40'),
        DeclareLaunchArgument(
            'use_collision_monitor', default_value='false', choices=['true', 'false'],
            description='仅 navigation 模式使用；完成独立验收前默认关闭'),
        DeclareLaunchArgument('use_keepout', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'livox_config',
            default_value=os.path.join(
                get_package_share_directory('livox_ros_driver2'), 'config',
                'G1_MID360s_config.json')),
        rosbridge_launch,
        sensor_launch,
        mapping_launch,
        localization_launch,
        navigation_launch,
        backend_nodes,
    ])
