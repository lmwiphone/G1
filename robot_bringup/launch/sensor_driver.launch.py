#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import ResetLaunchConfigurations
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
    start_realsense = LaunchConfiguration('start_realsense')

    return LaunchDescription([

        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        # 使用序列号绑定相机，与USB端口和/dev/video*编号无关。
        DeclareLaunchArgument(
            'realsense_serial_no', default_value="'347622073141'"),
        DeclareLaunchArgument('realsense_initial_reset', default_value='false'),
        DeclareLaunchArgument(
            'realsense_enable_color', default_value='true', choices=['true', 'false'],
            description='点云不再需要 color；纯避障场景可设 false 省带宽'),
        DeclareLaunchArgument('realsense_config', default_value=os.path.join(
            get_package_share_directory('robot_bringup'), 'param',
            'realsense_g1.yaml')),
        DeclareLaunchArgument('livox_config', default_value=os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'config',
            'G1_MID360s_config.json')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            condition=IfCondition(start_livox),
            launch_arguments={'livox_config': LaunchConfiguration('livox_config')}.items(),
        ),
        # 该版本 rs_launch.py 会把上下文中所有 launch 参数当作相机参数检查。
        # 在局部作用域中清除 robot.launch.py 的 mode/use_sim_time 等参数，
        # 只传入 RealSense 官方支持的参数，避免无意义的 unsupported 警告。
        GroupAction(
            condition=IfCondition(start_realsense),
            actions=[
                ResetLaunchConfigurations({
                'camera_namespace': 'camera',
                'camera_name': 'camera',
                'serial_no': LaunchConfiguration('realsense_serial_no'),
                'initial_reset': LaunchConfiguration('realsense_initial_reset'),
                'config_file': LaunchConfiguration('realsense_config'),
                'enable_depth': 'true',
                # 点云已在 realsense_g1.yaml 里关闭纹理映射（stream_filter=0），
                # 因此点云不再依赖 color 流。这里保留 color 开关供调试使用，
                # 只做避障时可以 realsense_enable_color:=false 省一半 USB 带宽。
                'enable_color': LaunchConfiguration('realsense_enable_color'),
                'pointcloud.enable': 'true',
                'pointcloud.allow_no_texture_points': 'true',
                'align_depth.enable': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'publish_tf': 'true',
                'output': 'screen',
                }),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch', 'rs_launch.py')),
                ),
            ],
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
