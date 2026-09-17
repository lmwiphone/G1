"""统一导航入口；不启动 SLAM、URDF 或任何 TF 发布器。"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from pathlib import Path


def generate_launch_description():
    bridge = Path(get_package_share_directory('g1_nav_bridge'))
    nav = Path(get_package_share_directory('aid_navigation2'))
    args = ['map', 'floor_z', 'base_floor_z', 'cloud_topic', 'sensor_frame',
            'use_realsense_obstacles', 'realsense_topic', 'robot_radius',
            'use_collision_monitor', 'use_keepout']
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/opt/G1/lighting_ws/data/new_map/map.yaml',
                             description='导航栅格地图 YAML 的绝对路径，由 map_server 加载'),
        DeclareLaunchArgument('floor_z', description='map 坐标下实测地面 Z，不能用雷达安装高度直接代替'),
        DeclareLaunchArgument('base_floor_z', default_value='0.0',
                             description='当前 URDF 的 base_link 为地面投影，地面 Z=0'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/points'),
        DeclareLaunchArgument('sensor_frame', default_value='mid360_link'),
        DeclareLaunchArgument('use_realsense_obstacles', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('realsense_topic', default_value='/camera/camera/depth/color/points'),
        DeclareLaunchArgument('robot_radius', default_value='0.40'),
        DeclareLaunchArgument('use_collision_monitor', default_value='true',
                             choices=['true', 'false'], description='false：跳过额外碰撞停车，桥接直接订阅 /cmd_vel'),
        DeclareLaunchArgument('use_keepout', default_value='false', choices=['true', 'false'],
                             description='有真实禁行区 mask 发布端时才开启'),
        DeclareLaunchArgument('start_bridge', default_value='true',
                             description='robot.launch 已启动 bridge 时必须设为 false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bridge/'launch/nav_bridge.launch.py')),
                                 launch_arguments={'cmd_vel_topic': PythonExpression([
                                     "'/cmd_vel_safe' if '", LaunchConfiguration('use_collision_monitor'),
                                     "' == 'true' else '/cmd_vel'"]) }.items(),
                                 condition=IfCondition(LaunchConfiguration('start_bridge'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/'launch/g1_navigation_direct.launch.py')),
                                 launch_arguments={k: LaunchConfiguration(k) for k in args}.items()),
    ])
