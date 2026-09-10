"""G1 direct-map profile; no TF publishers, URDF or fabricated odometry.

Adapted from the reviewed g1_nav_integration parameter organization (Apache-2.0).
Retains this workspace's planner, BT and keepout interfaces.
"""
from copy import deepcopy
import math


def make_configs(stock, collision, *, floor_z, base_floor_z, cloud_topic,
                 sensor_frame, robot_radius=0.40):
    if not all(math.isfinite(v) for v in (floor_z, base_floor_z, robot_radius)):
        raise ValueError('Ground heights and radius must be finite')
    if robot_radius <= 0 or not cloud_topic or not sensor_frame:
        raise ValueError('Positive robot_radius, cloud_topic and sensor_frame required')
    keep = ('bt_navigator', 'controller_server', 'local_costmap', 'global_costmap',
            'planner_server', 'smoother_server', 'behavior_server', 'velocity_smoother',
            'map_server', 'costmap_filter_info_server')
    nav = {k: deepcopy(stock[k]) for k in keep if k in stock}

    def frames(value):
        if isinstance(value, dict):
            for k, v in value.items():
                if k == 'use_sim_time':
                    value[k] = False
                elif k in ('robot_base_frame', 'base_frame_id'):
                    value[k] = 'base_link'
                else:
                    frames(v)
        elif isinstance(value, list):
            for v in value:
                frames(v)
    frames(nav)
    for name in ('local_costmap', 'global_costmap'):
        p = nav[name][name]['ros__parameters']
        p.update(global_frame='map', robot_base_frame='base_link', robot_radius=robot_radius,
                 resolution=0.05, transform_tolerance=0.20)
        p.pop('footprint', None)
        p['plugins'] = (['static_layer'] if name == 'global_costmap' else []) + [
            'obstacle_layer', 'keepout_layer', 'inflation_layer']
        p.pop('voxel_layer', None)
        p.pop('stvl_voxel_layer', None)
        p['obstacle_layer'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
            'combination_method': 1, 'max_obstacle_height': floor_z + 1.8,
            'observation_sources': 'livox',
            'livox': {'topic': cloud_topic, 'data_type': 'PointCloud2',
                      'sensor_frame': sensor_frame, 'marking': True, 'clearing': True,
                      'min_obstacle_height': floor_z + 0.10,
                      'max_obstacle_height': floor_z + 1.8,
                      'obstacle_min_range': 0.25, 'obstacle_max_range': 4.0,
                      'raytrace_min_range': 0.25, 'raytrace_max_range': 5.0}}
        p['inflation_layer'].update(inflation_radius=max(0.60, robot_radius + 0.20))
        if name == 'local_costmap':
            p.update(width=6, height=6, rolling_window=True)
    behavior = nav['behavior_server']['ros__parameters']
    behavior.update(local_frame='map', global_frame='map', enable_stamped_cmd_vel=False,
                    max_rotational_vel=0.25, min_rotational_vel=0.10, rotational_acc_lim=0.60)
    controller = nav['controller_server']['ros__parameters']
    controller.update(enable_stamped_cmd_vel=False, min_y_velocity_threshold=0.001)
    # Preserve the existing forward/turn-only command envelope, not a new lateral gait.
    controller['FollowPath'].update(vx_max=0.15, vx_min=0.0, vy_max=0.0,
                                     wz_max=0.25, visualize=False)
    smooth = nav['velocity_smoother']['ros__parameters']
    smooth.update(feedback='OPEN_LOOP', enable_stamped_cmd_vel=False,
                  max_velocity=[0.15, 0.0, 0.25], min_velocity=[-0.15, 0.0, -0.25],
                  max_accel=[0.35, 0.35, 0.60], max_decel=[-0.35, -0.35, -0.60],
                  velocity_timeout=0.20)
    smooth.pop('odom_topic', None)
    smooth.pop('odom_duration', None)
    # Controller/BT may still consume velocity observations depending on their implementation.
    # We do not invent zero odometry or derive velocity from globally jumping TF here.
    safety = {'collision_monitor': deepcopy(collision['collision_monitor'])}
    c = safety['collision_monitor']['ros__parameters']
    c.update(use_sim_time=False, base_frame_id='base_link', odom_frame_id='map',
             base_shift_correction=False, enable_stamped_cmd_vel=False,
             cmd_vel_in_topic='/cmd_vel', cmd_vel_out_topic='/cmd_vel_safe',
             source_timeout=0.35, observation_sources=['pointcloud1'])
    c['pointcloud1'] = {'type': 'pointcloud', 'topic': cloud_topic,
                        'min_height': base_floor_z + 0.10,
                        'max_height': base_floor_z + 1.8, 'enabled': True}
    c['polygons'] = ['SafetyZone']
    c.pop('FootprintApproach', None)
    # Fixed stop zone works before a local footprint is published and for zero lateral speed.
    c['SafetyZone'] = {'type': 'circle', 'action_type': 'stop',
                       'radius': robot_radius + 0.20, 'min_points': 4,
                       'visualize': True, 'enabled': True}
    return nav, safety
