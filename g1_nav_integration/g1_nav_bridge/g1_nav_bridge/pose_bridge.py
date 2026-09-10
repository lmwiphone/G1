import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from .geometry import transform, quaternion, body_twist, initial_tracking_pose


def pose_matrix(p):
    return transform([p.position.x, p.position.y, p.position.z],
                     [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def transform_matrix(t):
    return transform([t.translation.x, t.translation.y, t.translation.z],
                     [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])


def fill_pose(p, t):
    p.position.x, p.position.y, p.position.z = map(float, t[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quaternion(t))


class PoseBridge(Node):
    def __init__(self):
        super().__init__('g1_pose_bridge')
        self.tracking = self.declare_parameter('tracking_frame', 'lightning_tracking').value
        self.base = self.declare_parameter('base_frame', 'pelvis').value
        self.max_age = float(self.declare_parameter('max_pose_age', 0.35).value)
        if self.max_age <= 0:
            raise ValueError('max_pose_age must be positive')
        self.buffer = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/lightning/odom', 10)
        self.ready_pub = self.create_publisher(Bool, '/g1_nav/pose_valid', 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/lightning/initialpose', 10)
        self.global_queue = deque(maxlen=80)
        self.local_queue = deque(maxlen=80)
        self.initial_queue = deque(maxlen=2)
        self.history = deque(maxlen=100)
        self.last_global = None
        self.last_local = None
        self.last_stamp = {'map': -1, 'local': -1}
        self.last_global_wall = -1e9
        self.last_warning = -1e9
        self.create_subscription(PoseStamped, '/lightning/map_pose',
                                 lambda m: self.enqueue(self.global_queue, m, 'map'), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/lightning/lio_pose',
                                 lambda m: self.enqueue(self.local_queue, m, 'lightning_odom'), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 lambda m: self.initial_queue.append((time.monotonic(), m)), 10)
        self.create_timer(0.01, self.process)

    def warn(self, text):
        if time.monotonic() - self.last_warning > 2:
            self.get_logger().warning(text)
            self.last_warning = time.monotonic()

    def enqueue(self, queue, msg, frame):
        if msg.header.frame_id != frame:
            self.warn(f'Expected pose parent {frame}; received {msg.header.frame_id}')
            return
        queue.append((time.monotonic(), msg))

    def relative_at(self, stamp):
        t = self.buffer.lookup_transform(self.tracking, self.base, Time.from_msg(stamp))
        return transform_matrix(t.transform)

    def process_queue(self, queue, kind):
        while queue:
            receipt, msg = queue[0]
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if time.monotonic() - receipt > self.max_age or age > self.max_age or age < -0.1:
                queue.popleft()
                self.warn('Discarding stale/future pose: check sensor and ROS clocks')
                continue
            if stamp.nanoseconds <= self.last_stamp[kind]:
                queue.popleft()
                self.warn('Discarding duplicate/backward pose timestamp; restart after resetting a bag clock')
                continue
            try:
                tracking_to_base = self.relative_at(msg.header.stamp)
                track_pose = pose_matrix(msg.pose)
                base_pose = track_pose @ tracking_to_base
            except TransformException:
                # Wait for actual joint-state TF at the measurement time, not latest TF.
                break
            except ValueError as e:
                queue.popleft()
                self.warn(str(e))
                continue
            queue.popleft()
            self.last_stamp[kind] = stamp.nanoseconds
            if kind == 'map':
                self.last_global = (msg, track_pose)
                self.last_global_wall = time.monotonic()
                out = TransformStamped()
                out.header = msg.header
                out.child_frame_id = self.base
                out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = map(float, base_pose[:3, 3])
                out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = map(float, quaternion(base_pose))
                self.broadcaster.sendTransform(out)
            else:
                self.last_local = (msg, track_pose)
                t = stamp.nanoseconds * 1e-9
                self.history.append((t, base_pose))
                while len(self.history) > 2 and t - self.history[1][0] >= 0.08:
                    self.history.popleft()
                old_t, old_pose = self.history[0]
                if not 0.04 <= t - old_t <= 0.5:
                    if t - old_t > 0.5:
                        self.history.clear()
                    continue
                lin, ang = body_twist(old_pose, base_pose, t - old_t)
                odom = Odometry()
                odom.header = msg.header
                odom.child_frame_id = self.base
                fill_pose(odom.pose.pose, base_pose)
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, lin)
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, ang)
                self.odom_pub.publish(odom)

    def process(self):
        self.process_queue(self.local_queue, 'local')
        self.process_queue(self.global_queue, 'map')
        valid = time.monotonic() - self.last_global_wall < self.max_age
        self.ready_pub.publish(Bool(data=valid))
        if not self.initial_queue:
            return
        receipt, initial = self.initial_queue[0]
        if time.monotonic() - receipt > 2:
            self.initial_queue.popleft()
            self.warn('Initial pose expired while waiting for current LIO and joint TF')
            return
        current = self.last_global if valid else self.last_local
        if current is None:
            return
        msg, tracking_pose = current
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if age > self.max_age or age < -0.1:
            return
        if initial.header.frame_id != 'map':
            self.initial_queue.popleft()
            self.warn('RViz initial pose must use frame map')
            return
        try:
            new_pose = initial_tracking_pose(pose_matrix(initial.pose.pose), tracking_pose,
                                            self.relative_at(msg.header.stamp))
            output = PoseWithCovarianceStamped()
            output.header.frame_id = 'map'
            output.header.stamp = self.get_clock().now().to_msg()
            fill_pose(output.pose.pose, new_pose)
            # Lightning's current initialization API does not consume covariance.
            self.initial_pub.publish(output)
            self.last_global = None
            self.last_global_wall = -1e9
            self.global_queue.clear()
            self.initial_queue.popleft()
        except (TransformException, ValueError) as e:
            self.warn(str(e))


def main():
    rclpy.init()
    node = PoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
