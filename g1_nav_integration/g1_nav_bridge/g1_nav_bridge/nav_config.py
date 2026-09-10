from copy import deepcopy


def make_nav_config(stock, cloud_topic, floor_z, map_yaml):
    keep = ['bt_navigator', 'controller_server', 'local_costmap', 'global_costmap',
            'planner_server', 'smoother_server', 'behavior_server', 'velocity_smoother', 'collision_monitor']
    config = {k: deepcopy(stock[k]) for k in keep}
    def params(name):
        return config[name]['ros__parameters']
    params('bt_navigator').update(global_frame='map', robot_base_frame='pelvis', odom_topic='/lightning/odom')
    controller = params('controller_server')
    controller.update(odom_topic='/lightning/odom', min_y_velocity_threshold=0.001, enable_stamped_cmd_vel=False)
    controller['progress_checker'].update(required_movement_radius=0.15, movement_time_allowance=15.0)
    controller['FollowPath'].update(motion_model='Omni', vx_max=0.20, vx_min=-0.15, vy_max=0.10,
                                    wz_max=0.40, ax_max=0.30, ax_min=-0.40, ay_max=0.20, ay_min=-0.30,
                                    az_max=0.60, vx_std=0.08, vy_std=0.05, wz_std=0.15,
                                    visualize=False, batch_size=1000)
    for name in ['local_costmap', 'global_costmap']:
        p = config[name][name]['ros__parameters']
        p.update(global_frame='map', robot_base_frame='pelvis', robot_radius=0.40,
                 resolution=0.05, transform_tolerance=0.20)
        p['plugins'] = (['static_layer'] if name == 'global_costmap' else []) + ['obstacle_layer', 'inflation_layer']
        p.pop('voxel_layer', None)
        p['obstacle_layer'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
            'max_obstacle_height': floor_z+1.8, 'observation_sources': 'cloud',
            'cloud': {'topic': cloud_topic, 'data_type': 'PointCloud2', 'sensor_frame': 'mid360_link',
                      'marking': True, 'clearing': True, 'min_obstacle_height': floor_z+0.10,
                      'max_obstacle_height': floor_z+1.8, 'obstacle_min_range': 0.25,
                      'obstacle_max_range': 4.0, 'raytrace_min_range': 0.25, 'raytrace_max_range': 5.0}}
        p['inflation_layer'].update(inflation_radius=0.60, cost_scaling_factor=3.0)
        if name == 'local_costmap':
            p.update(width=6, height=6, rolling_window=True, update_frequency=10.0)
    params('behavior_server').update(local_frame='map', global_frame='map', robot_base_frame='pelvis',
                                     max_rotational_vel=0.40, min_rotational_vel=0.10,
                                     rotational_acc_lim=0.60, enable_stamped_cmd_vel=False)
    params('velocity_smoother').update(feedback='CLOSED_LOOP', odom_topic='/lightning/odom', odom_duration=0.10,
                                       max_velocity=[0.20, 0.10, 0.40], min_velocity=[-0.15, -0.10, -0.40],
                                       max_accel=[0.30, 0.20, 0.60], max_decel=[-0.40, -0.30, -0.60],
                                       velocity_timeout=0.25, enable_stamped_cmd_vel=False)
    c = params('collision_monitor')
    c.update(base_frame_id='pelvis', odom_frame_id='map', base_shift_correction=False,
             cmd_vel_in_topic='cmd_vel_smoothed', cmd_vel_out_topic='cmd_vel',
             enable_stamped_cmd_vel=False, observation_sources=['cloud'])
    c.pop('scan', None)
    c['cloud'] = {'type': 'pointcloud', 'topic': cloud_topic, 'min_height': -0.45,
                  'max_height': 1.2, 'enabled': True}
    config['map_server'] = {'ros__parameters': {'yaml_filename': str(map_yaml), 'topic_name': 'map', 'frame_id': 'map'}}
    return config
