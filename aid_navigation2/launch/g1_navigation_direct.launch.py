"""Compose existing Nav2/safety entries using map -> base_link. No SLAM/TF/drive launch."""
import importlib.util
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    nav_share = Path(get_package_share_directory('aid_navigation2'))
    robot_share = Path(get_package_share_directory('robot_bringup'))
    spec = importlib.util.spec_from_file_location('g1_config', nav_share/'launch/g1_config.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    map_file = Path(get('map')).expanduser().resolve()
    if not map_file.is_file():
        raise ValueError('Supply an existing saved map YAML')
    nav, safety = module.make_configs(
        yaml.safe_load((nav_share/'param/nav2_params.yaml').read_text()),
        yaml.safe_load((robot_share/'param/collision_monitor_params.yaml').read_text()),
        floor_z=float(get('floor_z')), base_floor_z=float(get('base_floor_z')),
        cloud_topic=get('cloud_topic'), sensor_frame=get('sensor_frame'),
        robot_radius=float(get('robot_radius')))
    # Retain generated parameters for diagnosis; no overwrite of source configs.
    output = Path(tempfile.mkdtemp(prefix='g1_direct_nav_'))
    nav_file, safety_file = output/'nav.yaml', output/'collision.yaml'
    nav_file.write_text(yaml.safe_dump(nav, sort_keys=False))
    safety_file.write_text(yaml.safe_dump(safety, sort_keys=False))
    return [
        LogInfo(msg=f'G1 direct map profile: {output}; robot motion bridge must remain disabled until validation.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share/'launch/navigation2.launch.py')),
            launch_arguments={'map': str(map_file), 'params_file': str(nav_file),
                              'use_sim_time': 'false', 'use_composition': 'False'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(robot_share/'launch/collision_monitor.launch.py')),
            launch_arguments={'params_file': str(safety_file), 'use_sim_time': 'false'}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Saved navigation map YAML'),
        DeclareLaunchArgument('floor_z', description='Measured ground Z in map coordinates'),
        DeclareLaunchArgument('base_floor_z', description='Measured ground Z in base_link at test stance'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/points'),
        DeclareLaunchArgument('sensor_frame', default_value='mid360_link'),
        DeclareLaunchArgument('robot_radius', default_value='0.40', description='Initial test envelope; verify on robot'),
        OpaqueFunction(function=setup)])
