import ast
import importlib.util
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent
spec = importlib.util.spec_from_file_location('g1_config', ROOT/'launch/g1_config.py')
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)


class G1ProfileTests(unittest.TestCase):
    def setUp(self):
        self.stock = yaml.safe_load((ROOT/'param/nav2_params.yaml').read_text())
        self.collision = yaml.safe_load((SRC/'robot_bringup/param/collision_monitor_params.yaml').read_text())
        self.nav, self.safe = config.make_configs(self.stock, self.collision,
            floor_z=-1.2, base_floor_z=-0.7, cloud_topic='/livox/points', sensor_frame='mid360_link',
            ros_distro='humble')

    def test_direct_frames(self):
        for name in ('local_costmap', 'global_costmap'):
            p = self.nav[name][name]['ros__parameters']
            self.assertEqual((p['global_frame'], p['robot_base_frame']), ('map', 'base_link'))
        b = self.nav['behavior_server']['ros__parameters']
        self.assertEqual((b['local_frame'], b['global_frame']), ('map', 'map'))
        c = self.safe['collision_monitor']['ros__parameters']
        self.assertEqual(c['odom_frame_id'], 'map')
        self.assertFalse(c['base_shift_correction'])
        self.assertNotIn('amcl', self.nav)

    def test_negative_z_obstacles_and_distinct_height_frames(self):
        local = self.nav['local_costmap']['local_costmap']['ros__parameters']
        self.assertTrue(local['obstacle_layer']['footprint_clearing_enabled'])
        p = local['obstacle_layer']['livox']
        self.assertAlmostEqual(p['min_obstacle_height'], -1.1)
        self.assertLess(p['min_obstacle_height'], -0.9)  # 30 cm box above test floor
        self.assertGreater(p['min_obstacle_height'], -1.2)  # floor rejected
        self.assertEqual(p['data_type'], 'PointCloud2')
        self.assertEqual(p['sensor_frame'], 'mid360_link')
        self.assertTrue(p['marking'] and p['clearing'])
        c = self.safe['collision_monitor']['ros__parameters']['pointcloud1']
        self.assertAlmostEqual(c['min_height'], -0.6)

    def test_realsense_is_opt_in_and_uses_message_frame(self):
        for name in ('local_costmap', 'global_costmap'):
            params = self.nav[name][name]['ros__parameters']
            layer = params['obstacle_layer']
            self.assertEqual(layer['observation_sources'], 'livox')
            self.assertNotIn('realsense', layer)
            self.assertNotIn('stvl_voxel_layer', params)
        nav, _ = config.make_configs(
            self.stock, self.collision, floor_z=-1.2, base_floor_z=0,
            cloud_topic='/livox/points', sensor_frame='mid360_link',
            realsense_topic='/camera/camera/depth/color/points',
            ros_distro='jazzy')
        local = nav['local_costmap']['local_costmap']['ros__parameters']
        self.assertEqual(local['obstacle_layer']['observation_sources'], 'livox')
        self.assertIn('stvl_voxel_layer', local['plugins'])
        stvl = local['stvl_voxel_layer']
        self.assertEqual(stvl['plugin'],
                         'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
        self.assertEqual(stvl['observation_sources'],
                         'realsense_mark realsense_clear')
        self.assertEqual(stvl['realsense_mark']['topic'],
                         '/camera/camera/depth/color/points')
        self.assertEqual(stvl['realsense_mark']['filter'], 'voxel')
        self.assertEqual(stvl['realsense_mark']['voxel_min_points'], 2)
        self.assertNotIn('sensor_frame', stvl['realsense_mark'])
        self.assertAlmostEqual(stvl['realsense_clear']['min_obstacle_height'], -1.15)
        self.assertAlmostEqual(stvl['realsense_clear']['max_obstacle_height'], 0.6)

        global_params = nav['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(global_params['obstacle_layer']['observation_sources'], 'livox')
        self.assertNotIn('stvl_voxel_layer', global_params['plugins'])
        self.assertNotIn('stvl_voxel_layer', global_params)

    def test_costmap_rates_match_local_control_without_duplicate_rgbd(self):
        local = self.nav['local_costmap']['local_costmap']['ros__parameters']
        global_params = self.nav['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(local['update_frequency'], 10.0)
        self.assertEqual(local['publish_frequency'], 5.0)
        self.assertEqual(global_params['update_frequency'], 1.0)
        self.assertEqual(global_params['publish_frequency'], 1.0)
        self.assertEqual(local['obstacle_layer']['observation_sources'], 'livox')
        self.assertEqual(global_params['obstacle_layer']['observation_sources'], 'livox')

    def test_frontend_keepout_preserved_and_no_source_mutation(self):
        for name in ('local_costmap', 'global_costmap'):
            p = self.nav[name][name]['ros__parameters']
            self.assertIn('keepout_layer', p['plugins'])
            self.assertEqual(p['keepout_layer'], self.stock[name][name]['ros__parameters']['keepout_layer'])
        self.assertEqual(self.nav['costmap_filter_info_server'], self.stock['costmap_filter_info_server'])
        self.assertEqual(self.stock['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame'], 'base_link')

    def test_safe_topics_open_loop_limits(self):
        for node in ('controller_server', 'bt_navigator'):
            self.assertEqual(self.nav[node]['ros__parameters']['odom_topic'], '/odom')
        s = self.nav['velocity_smoother']['ros__parameters']
        self.assertEqual(s['feedback'], 'OPEN_LOOP')
        self.assertNotIn('odom_topic', s)
        c = self.safe['collision_monitor']['ros__parameters']
        self.assertEqual((c['cmd_vel_in_topic'], c['cmd_vel_out_topic']), ('/cmd_vel', '/cmd_vel_safe'))
        self.assertEqual(c['SafetyZone']['action_type'], 'stop')
        configs = list((SRC/'g1_nav_bridge/config').glob('*.yaml'))
        bridge = next(yaml.safe_load(p.read_text())['g1_cmdvel_to_sport']['ros__parameters']
                      for p in configs if 'g1_cmdvel_to_sport' in yaml.safe_load(p.read_text()))
        self.assertEqual(bridge['cmd_vel_topic'], '/cmd_vel_safe')
        self.assertGreater(bridge['duration'], 0.0)
        self.assertEqual(s['max_velocity'], [0.5, 0.0, 0.9])
        self.assertEqual(s['min_velocity'], [0.0, 0.0, -0.9])
        self.assertEqual(s['deadband_velocity'], [0.3, 0.0, 0.8])

    def test_reject_missing_or_invalid_geometry(self):
        with self.assertRaises(ValueError):
            config.make_configs(self.stock, self.collision, floor_z=float('nan'),
                                base_floor_z=0, cloud_topic='/points', sensor_frame='lidar')

    def test_jazzy_plugin_names(self):
        nav, _ = config.make_configs(self.stock, self.collision, floor_z=-1.2,
            base_floor_z=0, cloud_topic='/livox/points', sensor_frame='mid360_link', ros_distro='jazzy')
        self.assertEqual(nav['planner_server']['ros__parameters']['GridBased']['plugin'],
                         'nav2_smac_planner::SmacPlanner2D')
        self.assertEqual(nav['behavior_server']['ros__parameters']['spin']['plugin'], 'nav2_behaviors::Spin')
        self.assertNotIn('plugin_lib_names', nav['bt_navigator']['ros__parameters'])
        self.assertEqual(nav['controller_server']['ros__parameters']['progress_checker_plugins'], ['progress_checker'])

    def test_controller_envelope_and_humble_checker(self):
        p = self.nav['controller_server']['ros__parameters']
        self.assertEqual(p['progress_checker_plugin'], 'progress_checker')
        self.assertAlmostEqual(p['FollowPath']['model_dt'], 1. / p['controller_frequency'])
        self.assertEqual(p['FollowPath']['vx_min'], 0.0)
        self.assertEqual(p['FollowPath']['vx_max'], 0.5)
        self.assertEqual(p['FollowPath']['wz_max'], 0.9)
        behavior = self.nav['behavior_server']['ros__parameters']
        self.assertEqual(behavior['max_rotational_vel'], 0.9)
        self.assertEqual(behavior['min_rotational_vel'], 0.8)

    def test_launch_syntax_and_required_heights(self):
        for p in (ROOT/'launch').glob('*.py'):
            ast.parse(p.read_text())
        text = (ROOT/'launch/g1_navigation_direct.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument('floor_z', description=", text)
        self.assertIn("DeclareLaunchArgument('base_floor_z', default_value='0.0'", text)
        self.assertNotIn('static_transform_publisher', text)
        self.assertNotIn("package='lightning'", text)

    def test_reference_isolation_and_map_resolution(self):
        self.assertFalse((SRC/'g1_nav_integration').exists())
        source = (SRC/'lightning-lm/src/core/system/slam.cc').read_text()
        self.assertIn('YAML::Key << "resolution" << YAML::Value << map.info.resolution', source)
        self.assertNotIn('YAML::Key << "resolution" << YAML::Value << float(0.05)', source)


if __name__ == '__main__':
    unittest.main(verbosity=2)
