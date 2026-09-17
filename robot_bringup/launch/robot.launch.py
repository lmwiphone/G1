#!/usr/bin/env python3
"""G1 唯一顶层入口。

本文件只负责编排已有的原子 launch，不在这里重复创建 Livox、Lightning、
Nav2、运动桥或 collision_monitor 节点。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _is_mode(*modes):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('requested_mode'), "' in ", repr(tuple(modes))
    ]))


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    bridge_share = get_package_share_directory('g1_nav_bridge')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox = LaunchConfiguration('start_livox')
    start_realsense = LaunchConfiguration('start_realsense')
    map_dir = LaunchConfiguration('map_dir')
    map_save_root = LaunchConfiguration('map_save_root')
    with_ui = LaunchConfiguration('with_ui')
    with_2dui = LaunchConfiguration('with_2dui')
    start_rviz = LaunchConfiguration('start_rviz')

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sensor_driver.launch.py')),
        launch_arguments={
            'start_livox': start_livox,
            'livox_config': LaunchConfiguration('livox_config'),
            'start_realsense': start_realsense,
            'realsense_serial_no': LaunchConfiguration('realsense_serial_no'),
            'realsense_initial_reset': LaunchConfiguration('realsense_initial_reset'),
            'realsense_enable_color': LaunchConfiguration('realsense_enable_color'),
        }.items(),
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'battery_bridge.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_battery_bridge')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
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
            'map_save_root': map_save_root,
            'with_ui': with_ui,
            'with_2dui': with_2dui,
            'start_rviz': start_rviz,
            'pub_registered_scan': LaunchConfiguration('pub_registered_scan'),
            'start_map_transform': PythonExpression([
                "'false' if '", LaunchConfiguration('start_backend'), "' == 'true' else 'true'"
            ]),
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
            'pub_registered_scan': LaunchConfiguration('pub_registered_scan'),
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
            'use_realsense_obstacles': LaunchConfiguration('use_realsense_obstacles'),
            'realsense_topic': LaunchConfiguration('realsense_topic'),
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
                 # 定位/Nav2 已由本顶层 launch 创建。禁止状态管理节点再经由
                 # launch_manager 启动第二套相同进程。
                 parameters=[{
                     'use_sim_time': use_sim_time,
                     'manage_stack': False,
                     'startup_mode': LaunchConfiguration('requested_mode'),
                 }]),
            Node(package='aid_robot_py', executable='map_transform_node',
                 name='map_transform_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='aid_robot_py', executable='launch_manager_node',
                 name='launch_manager_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            # 禁行区地图节点只在 use_keepout:=true 时启动。
            # keepout 关闭时没有 keepout_filter_map 的订阅者，该节点每收到一帧 /map
            # 就会去调用不存在的 get_current_forbidden 服务，产生
            # "get_forbidden_client return false" 噪声；并且它与导航栈另一份同名
            # 实例共存时会触发 "Publisher already registered for node name"。
            Node(package='robot_bringup', executable='forbidden_map_create_node',
                 condition=IfCondition(LaunchConfiguration('use_keepout')),
                 name='forbidden_map_create_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='aid_robot_py', executable='waypoint_manage_node',
                 name='waypoint_manage_node', prefix=['taskset -c 3-7'], output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
        ],
    )

    # Thor 当前经过验证并实际使用的 Lightning/2D 地图目录。
    default_map_dir = '/opt/G1/lighting_ws/data/new_map'
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='navigation',
            choices=['base', 'mapping', 'localization', 'navigation'],
            description='互斥运行模式；默认启动定位、Nav2 与运动桥'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('start_battery_bridge', default_value='true'),
        # 使用序列号绑定相机，与USB端口和/dev/video*编号无关。
        DeclareLaunchArgument(
            'realsense_serial_no', default_value="'347622073141'"),
        DeclareLaunchArgument('realsense_initial_reset', default_value='false'),
        DeclareLaunchArgument(
            'realsense_enable_color', default_value='true', choices=['true', 'false'],
            description='点云不再需要 color；纯避障场景可设 false 省带宽'),
        DeclareLaunchArgument('start_rosbridge', default_value='true'),
        DeclareLaunchArgument('start_backend', default_value='true'),
        DeclareLaunchArgument('with_ui', default_value='false'),
        DeclareLaunchArgument('with_2dui', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('map_dir', default_value=default_map_dir),
        DeclareLaunchArgument('map_save_root', default_value=os.path.expanduser('~/maps')),
        DeclareLaunchArgument('nav_map', default_value=default_map_dir + '/map.yaml'),
        DeclareLaunchArgument('floor_height', default_value=''),
        DeclareLaunchArgument('min_obstacle_height', default_value=''),
        DeclareLaunchArgument('max_obstacle_height', default_value=''),
        DeclareLaunchArgument('floor_z', default_value='0.0'),
        DeclareLaunchArgument('base_floor_z', default_value='0.0'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/points'),
        DeclareLaunchArgument('sensor_frame', default_value='mid360_link'),
        DeclareLaunchArgument(
            'realsense_topic',
            default_value='/camera/camera/depth/color/points'),
        DeclareLaunchArgument(
            'use_realsense_obstacles', default_value='true',
            choices=['true', 'false'],
            description='将D435点云经STVL加入Nav2局部障碍层'),
        DeclareLaunchArgument('robot_radius', default_value='0.40'),
        DeclareLaunchArgument(
            'use_collision_monitor', default_value='false', choices=['true', 'false'],
            description='仅 navigation 模式使用；完成独立验收前默认关闭'),
        DeclareLaunchArgument('use_keepout', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'pub_registered_scan', default_value='true', choices=['true', 'false'],
            description='发布 map 系配准点云 /lightning/registered_scan（默认开启，便于在 RViz 直接看配准结果）'),
        DeclareLaunchArgument(
            'livox_config',
            default_value=os.path.join(
                get_package_share_directory('livox_ros_driver2'), 'config',
                'G1_MID360s_config.json')),
        # 子 launch（Lightning）内部也使用名为 mode 的参数，并会把它改成
        # localization/mapping。先保存用户请求的顶层模式，避免后续 Nav2
        # 条件和后台节点被子 launch 的同名参数污染。
        SetLaunchConfiguration('requested_mode', LaunchConfiguration('mode')),
        rosbridge_launch,
        sensor_launch,
        battery_launch,
        mapping_launch,
        localization_launch,
        navigation_launch,
        backend_nodes,
    ])
