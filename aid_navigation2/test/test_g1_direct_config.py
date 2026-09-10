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
            floor_z=-1.2, base_floor_z=-0.7, cloud_topic='/livox/points', sensor_frame='mid360_link')

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
        p = self.nav['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['livox']
        self.assertAlmostEqual(p['min_obstacle_height'], -1.1)
        self.assertLess(p['min_obstacle_height'], -0.9)  # 30 cm box above test floor
        self.assertGreater(p['min_obstacle_height'], -1.2)  # floor rejected
        self.assertEqual(p['data_type'], 'PointCloud2')
        self.assertEqual(p['sensor_frame'], 'mid360_link')
        self.assertTrue(p['marking'] and p['clearing'])
        c = self.safe['collision_monitor']['ros__parameters']['pointcloud1']
        self.assertAlmostEqual(c['min_height'], -0.6)

    def test_frontend_keepout_preserved_and_no_source_mutation(self):
        for name in ('local_costmap', 'global_costmap'):
            p = self.nav[name][name]['ros__parameters']
            self.assertIn('keepout_layer', p['plugins'])
            self.assertEqual(p['keepout_layer'], self.stock[name][name]['ros__parameters']['keepout_layer'])
        self.assertEqual(self.nav['costmap_filter_info_server'], self.stock['costmap_filter_info_server'])
        self.assertEqual(self.stock['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame'], 'base_footprint')

    def test_safe_topics_open_loop_limits(self):
        s = self.nav['velocity_smoother']['ros__parameters']
        self.assertEqual(s['feedback'], 'OPEN_LOOP')
        self.assertNotIn('odom_topic', s)
        c = self.safe['collision_monitor']['ros__parameters']
        self.assertEqual((c['cmd_vel_in_topic'], c['cmd_vel_out_topic']), ('/cmd_vel', '/cmd_vel_safe'))
        self.assertEqual(c['SafetyZone']['action_type'], 'stop')
        configs = list((SRC/'g1_nav_bridge/config').glob('*.yaml'))
        bridge = next(yaml.safe_load(p.read_text())['g1_cmdvel_to_sport']['ros__parameters']
                      for p in configs if 'g1_cmdvel_to_sport' in yaml.safe_load(p.read_text()))
        self.assertFalse(bridge['enabled_on_start'])
        self.assertEqual(bridge['cmd_vel_topic'], '/cmd_vel_safe')
        self.assertEqual(s['max_velocity'], [0.15, 0.0, 0.25])

    def test_reject_missing_or_invalid_geometry(self):
        with self.assertRaises(ValueError):
            config.make_configs(self.stock, self.collision, floor_z=float('nan'),
                                base_floor_z=0, cloud_topic='/points', sensor_frame='lidar')

    def test_launch_syntax_and_required_heights(self):
        for p in (ROOT/'launch').glob('*.py'):
            ast.parse(p.read_text())
        text = (ROOT/'launch/g1_navigation_direct.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument('floor_z', description=", text)
        self.assertIn("DeclareLaunchArgument('base_floor_z', description=", text)
        self.assertNotIn('static_transform_publisher', text)
        self.assertNotIn("package='lightning'", text)

    def test_reference_isolation_and_map_resolution(self):
        self.assertTrue((SRC/'g1_nav_integration/COLCON_IGNORE').exists())
        source = (SRC/'lightning-lm/src/core/system/slam.cc').read_text()
        self.assertIn('YAML::Key << "resolution" << YAML::Value << map.info.resolution', source)
        self.assertNotIn('YAML::Key << "resolution" << YAML::Value << float(0.05)', source)


if __name__ == '__main__':
    unittest.main(verbosity=2)
