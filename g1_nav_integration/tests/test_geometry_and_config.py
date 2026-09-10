import sys
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'g1_nav_bridge'))
from g1_nav_bridge.geometry import transform, quaternion, inverse, planar, initial_tracking_pose, body_twist
from g1_nav_bridge.model import prepare_description
from g1_nav_bridge.nav_config import make_nav_config


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    return transform([0, 0, 0], list(axis * math.sin(angle/2)) + [math.cos(angle/2)])


def urdf_transform(urdf, target, q):
    joints = {j.find('child').attrib['link']: j for j in ET.fromstring(urdf).findall('joint')}
    t = np.eye(4)
    while target != 'pelvis':
        j = joints[target]
        origin = j.find('origin')
        xyz = [0., 0., 0.] if origin is None else list(map(float, origin.get('xyz', '0 0 0').split()))
        roll, pitch, yaw = [0., 0., 0.] if origin is None else list(map(float, origin.get('rpy', '0 0 0').split()))
        a = axis_rotation([0,0,1], yaw) @ axis_rotation([0,1,0], pitch) @ axis_rotation([1,0,0], roll)
        a[:3, 3] = xyz
        if j.attrib['type'] != 'fixed':
            axis = list(map(float, j.find('axis').attrib['xyz'].split()))
            a = a @ axis_rotation(axis, q.get(j.attrib['name'], 0.0))
        t = a @ t
        target = j.find('parent').attrib['link']
    return t


class GeometryTest(unittest.TestCase):
    def test_quaternion_round_trip_and_pi(self):
        for axis in ([1,0,0], [0,1,0], [0,0,1], [1,2,3]):
            for angle in (0, .2, 1.57, math.pi, -math.pi, 3.13):
                t = axis_rotation(axis, angle)
                self.assertTrue(np.allclose(t, transform([0,0,0], quaternion(t)), atol=1e-9))

    def test_official_23_and_29_joint_chains(self):
        config = {'fasterlio': {'extrinsic_T': [.01, -.02, .03],
                                'extrinsic_R': axis_rotation([0,1,0], .1)[:3,:3].flatten().tolist()}}
        til = axis_rotation([0,1,0], .1)
        til[:3,3] = [.01, -.02, .03]
        for name in ('reference_g1_23dof.urdf', 'reference_g1_29dof.urdf'):
            original = (ROOT/'vendor'/name).read_text()
            model, active = prepare_description(original, config)
            for angles in ({}, {'waist_yaw_joint': .7, 'waist_roll_joint': .1, 'waist_pitch_joint': -.15}):
                tbl = urdf_transform(original, 'mid360_link', angles)
                tbi = urdf_transform(model, 'lightning_tracking', angles)
                self.assertTrue(np.allclose(tbi @ til, tbl, atol=1e-9))
                twb = axis_rotation([1,2,3], .5)
                twb[:3,3] = [2,3,.7]
                twi = twb @ tbi
                self.assertTrue(np.allclose(twi @ inverse(tbi), twb, atol=1e-9))
            self.assertIn('waist_yaw_joint', active)
            self.assertNotIn('<visual>', model)

    def test_joint_motion_does_not_move_stationary_pelvis(self):
        config = {'fasterlio': {'extrinsic_T': [0,0,0], 'extrinsic_R': np.eye(3).flatten().tolist()}}
        model, _ = prepare_description((ROOT/'vendor/reference_g1_29dof.urdf').read_text(), config)
        base = np.eye(4)
        base[:3,3] = [1,2,.7]
        recovered = []
        for yaw in (0, .1, .2):
            tbi = urdf_transform(model, 'lightning_tracking', {'waist_yaw_joint':yaw})
            recovered.append((base @ tbi) @ inverse(tbi))
        lin, ang = body_twist(recovered[0], recovered[-1], .1)
        self.assertTrue(np.allclose(lin, 0, atol=1e-8))
        self.assertTrue(np.allclose(ang, 0, atol=1e-8))

    def test_initial_pose_keeps_height_and_tilt(self):
        current = axis_rotation([1,0,0], .12)
        current[:3,3] = [3, 4, .9]
        tracking_to_base = axis_rotation([0,0,1], .3)
        tracking_to_base[:3,3] = [.1, .02, -.4]
        desired = axis_rotation([0,0,1], 1.2)
        desired[:3,3] = [7, 8, 0]
        result = initial_tracking_pose(desired, current, tracking_to_base)
        self.assertTrue(np.allclose(planar(result @ tracking_to_base), planar(desired)))
        self.assertAlmostEqual(result[2,3], current[2,3])
        self.assertTrue(np.allclose(result[2,:3], current[2,:3]))

    def test_velocity_in_body_frame_and_angle_wrap(self):
        old = axis_rotation([0,0,1], math.pi/2)
        new = old.copy()
        new[1,3] = .02
        lin, ang = body_twist(old, new, .1)
        self.assertTrue(np.allclose(lin, [.2,0,0], atol=1e-9))
        a = axis_rotation([0,0,1], math.pi-.01)
        b = axis_rotation([0,0,1], -math.pi+.01)
        _, ang = body_twist(a,b,.1)
        self.assertAlmostEqual(ang[2], .2, places=8)

    def test_invalid_inputs(self):
        for p,q in [([0,0,0],[0,0,0,0]), ([float('nan'),0,0],[0,0,0,1])]:
            with self.assertRaises(ValueError):
                transform(p,q)
        with self.assertRaises(ValueError):
            body_twist(np.eye(4), np.eye(4), 0)

    def test_nav_uses_consistent_frames_and_existing_cloud(self):
        stock = yaml.safe_load((ROOT/'g1_nav_bridge/config/nav2_jazzy_reference.yaml').read_text())
        cfg = make_nav_config(stock, '/user/existing_cloud', -1.2, '/maps/test/map.yaml')
        for name in ['local_costmap','global_costmap']:
            p = cfg[name][name]['ros__parameters']
            self.assertEqual(p['global_frame'], 'map')
            self.assertEqual(p['robot_base_frame'], 'pelvis')
            source = p['obstacle_layer']['cloud']
            self.assertEqual(source['topic'], '/user/existing_cloud')
            self.assertEqual(source['data_type'], 'PointCloud2')
            self.assertAlmostEqual(source['min_obstacle_height'], -1.1)
        self.assertEqual(cfg['controller_server']['ros__parameters']['FollowPath']['motion_model'], 'Omni')
        self.assertEqual(cfg['collision_monitor']['ros__parameters']['odom_frame_id'], 'map')
        self.assertNotIn('amcl', cfg)


if __name__ == '__main__':
    unittest.main(verbosity=2)
