import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from g1_nav_bridge.model import prepare_description
from g1_nav_bridge.nav_config import make_nav_config


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    urdf = Path(get('urdf')).expanduser().resolve()
    lm_yaml = Path(get('lightning_config')).expanduser().resolve()
    root = Path(get('lightning_root')).expanduser().resolve()
    mode = get('mode')
    if mode not in ('mapping', 'navigation'):
        raise ValueError('mode must be mapping or navigation')
    lm = yaml.safe_load(lm_yaml.read_text())
    description, active = prepare_description(urdf.read_text(), lm, get('cloud_alias'))
    output = Path(context.launch_configurations['ros_log_dir']) if 'ros_log_dir' in context.launch_configurations else Path(os.environ.get('ROS_LOG_DIR', str(Path.home()/'.ros/log')))
    output.mkdir(parents=True, exist_ok=True)
    nav_file = output / ('g1_nav_params_' + str(os.getpid()) + '.yaml')
    nodes = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                  parameters=[{'robot_description': description, 'publish_frequency': 100.0}], output='screen'),
             Node(package='g1_nav_bridge', executable='pose_bridge', output='screen')]
    if get('publish_joint_states').lower() == 'true':
        nodes.append(Node(package='g1_nav_bridge', executable='unitree_joint_state_bridge', output='screen',
                          parameters=[{'active_joints': active, 'lowstate_topic': get('lowstate_topic')}]))
    nodes.append(Node(package='lightning', executable='run_slam_online' if mode == 'mapping' else 'run_loc_online',
                      arguments=['--config=' + str(lm_yaml)], cwd=str(root), output='screen',
                      parameters=[{'publish_tf': False}]))
    if mode == 'mapping':
        return [LogInfo(msg='G1 mapping: driver and existing PointCloud2 output remain managed by your current launch.')] + nodes
    map_dir = Path(lm['system']['map_path'])
    if not map_dir.is_absolute():
        map_dir = root / map_dir
    map_yaml = Path(get('map_yaml')).expanduser() if get('map_yaml') else map_dir/'map.yaml'
    if not map_yaml.is_file():
        raise ValueError(f'Map file does not exist: {map_yaml}')
    cloud_topic = get('cloud_topic')
    if not cloud_topic:
        raise ValueError('Set cloud_topic to your existing PointCloud2 topic')
    if get('floor_z'):
        floor = float(get('floor_z'))
    elif lm['g2p5'].get('esti_floor', False):
        raise ValueError('With dynamic floor estimation, supply floor_z in saved map coordinates')
    else:
        floor = float(lm['g2p5']['floor_height'])
    share = Path(get_package_share_directory('g1_nav_bridge'))
    stock = yaml.safe_load((share/'config/nav2_jazzy_reference.yaml').read_text())
    nav_file.write_text(yaml.safe_dump(make_nav_config(stock, cloud_topic, floor, map_yaml), sort_keys=False))
    lifecycle_names = []
    components = [('nav2_map_server','map_server'), ('nav2_controller','controller_server'),
                  ('nav2_smoother','smoother_server'), ('nav2_planner','planner_server'),
                  ('nav2_behaviors','behavior_server'), ('nav2_bt_navigator','bt_navigator'),
                  ('nav2_velocity_smoother','velocity_smoother'), ('nav2_collision_monitor','collision_monitor')]
    for package, executable in components:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', 'cmd_vel_nav')]
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', 'cmd_vel_nav')]
        nodes.append(Node(package=package, executable=executable, name=executable,
                          parameters=[str(nav_file)], remappings=remaps, output='screen'))
        lifecycle_names.append(executable)
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_g1', parameters=[{'autostart': True, 'node_names': lifecycle_names}],
                      output='screen'))
    if get('start_drive').lower() == 'true':
        nodes.append(Node(package='g1_nav_bridge', executable='unitree_command_bridge', output='screen'))
    return [LogInfo(msg=f'Generated Nav2 parameters: {nav_file}')] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf', description='Absolute path to your official G1 URDF'),
        DeclareLaunchArgument('lightning_config', description='Your existing, compensated Lightning YAML'),
        DeclareLaunchArgument('lightning_root', description='Lightning source/work directory for relative data paths'),
        DeclareLaunchArgument('mode', default_value='navigation'),
        DeclareLaunchArgument('cloud_topic', default_value='', description='Existing PointCloud2 output'),
        DeclareLaunchArgument('cloud_alias', default_value='', description='Optional identity alias for a corrected sensor cloud; leave empty for map clouds'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('floor_z', default_value=''),
        DeclareLaunchArgument('publish_joint_states', default_value='true'),
        DeclareLaunchArgument('lowstate_topic', default_value='/lowstate'),
        DeclareLaunchArgument('start_drive', default_value='false', description='Start G1 velocity API bridge'),
        OpaqueFunction(function=setup)])
