import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Official unitree_ros2/example/src/src/g1/lowlevel/g1_low_level_example.cpp
JOINTS = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint',
    'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint',
    'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint',
    'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
    'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint']


class JointBridge(Node):
    def __init__(self):
        from unitree_hg.msg import LowState
        super().__init__('g1_joint_state_bridge')
        active = self.declare_parameter('active_joints', JOINTS).value
        unknown = set(active) - set(JOINTS)
        if unknown:
            raise ValueError(f'Unmapped URDF joints: {sorted(unknown)}; provide a complete external /joint_states instead')
        self.mapping = [(i, name) for i, name in enumerate(JOINTS) if name in active]
        self.stamp_offset = float(self.declare_parameter('joint_time_offset', 0.0).value)
        self.last_warning = 0.0
        self.last_tick = None
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        topic = self.declare_parameter('lowstate_topic', '/lowstate').value
        self.create_subscription(LowState, topic, self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        # Official q mapping is pitch/roll mode. Do not interpret A/B actuator values as joints.
        if msg.mode_pr != 0:
            if time.monotonic() - self.last_warning > 2:
                self.get_logger().error('LowState is not PR mode; no joint TF published from A/B values')
                self.last_warning = time.monotonic()
            return
        if self.last_tick is not None:
            delta = (int(msg.tick) - self.last_tick) & 0xffffffff
            if delta == 0 or delta > 0x7fffffff:
                return
        self.last_tick = msg.tick
        output = JointState()
        # LowState has no ROS Header; receipt timestamp is explicit, never a fabricated device epoch.
        output.header.stamp = (self.get_clock().now() + Duration(seconds=self.stamp_offset)).to_msg()
        for i, name in self.mapping:
            if i >= len(msg.motor_state) or not math.isfinite(msg.motor_state[i].q):
                return
            output.name.append(name)
            output.position.append(float(msg.motor_state[i].q))
            output.velocity.append(float(msg.motor_state[i].dq))
        self.pub.publish(output)


class CommandBridge(Node):
    def __init__(self):
        from unitree_api.msg import Request
        super().__init__('g1_command_bridge')
        self.Request = Request
        self.limit = [float(self.declare_parameter(n, d).value) for n, d in
                      [('max_vx', .20), ('max_vy', .10), ('max_wz', .40)]]
        self.timeout = float(self.declare_parameter('command_timeout', .25).value)
        if self.timeout <= 0 or any(x <= 0 or not math.isfinite(x) for x in self.limit):
            raise ValueError('Velocity limits and timeout must be finite and positive')
        self.last_command = -1e9
        self.last_valid = -1e9
        self.velocity = [0., 0., 0.]
        self.ever_commanded = False
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 1)
        self.create_subscription(Bool, '/g1_nav/pose_valid', self.valid, 10)
        self.create_timer(.05, self.send)

    def valid(self, msg):
        self.last_valid = time.monotonic() if msg.data else -1e9

    def command(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not all(math.isfinite(x) for x in values):
            self.velocity = [0., 0., 0.]
            self.last_command = -1e9
            return
        self.velocity = [max(-limit, min(limit, float(v))) for v, limit in zip(values, self.limit)]
        self.last_command = time.monotonic()
        self.ever_commanded = True

    def send(self, stop=False):
        if not self.ever_commanded:
            return
        now = time.monotonic()
        valid = not stop and now-self.last_command < self.timeout and now-self.last_valid < self.timeout
        request = self.Request()
        request.header.identity.id = time.time_ns()
        request.header.identity.api_id = 7105
        request.header.policy.noreply = True
        request.parameter = json.dumps({'velocity': self.velocity if valid else [0., 0., 0.],
                                        'duration': .20}, allow_nan=False)
        self.pub.publish(request)


def run(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(node, CommandBridge) and rclpy.ok():
            node.send(stop=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def joint_main():
    run(JointBridge)


def command_main():
    run(CommandBridge)
