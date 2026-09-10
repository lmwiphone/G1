#!/usr/bin/env python3
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from unitree_api.msg import Request, Response


ROBOT_API_ID_LOCO_SET_VELOCITY = 7105


class CmdVelToSport(Node):
    def __init__(self):
        super().__init__('g1_cmdvel_to_sport')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('sport_response_topic', '/api/sport/response')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('timeout_s', 0.20)
        self.declare_parameter('command_duration', 0.20)
        self.declare_parameter('max_vx', 0.15)
        self.declare_parameter('max_vy', 0.0)
        self.declare_parameter('max_wz', 0.25)
        self.declare_parameter('max_ax', 0.35)
        self.declare_parameter('max_ay', 0.35)
        self.declare_parameter('max_awz', 0.60)
        self.declare_parameter('enabled_on_start', False)

        param = lambda name: self.get_parameter(name).value
        self.rate_hz = float(param('rate_hz'))
        self.timeout_s = float(param('timeout_s'))
        self.command_duration = float(param('command_duration'))
        self.limits = [float(param('max_vx')), float(param('max_vy')), float(param('max_wz'))]
        self.accel_limits = [float(param('max_ax')), float(param('max_ay')), float(param('max_awz'))]
        self.enabled = bool(param('enabled_on_start'))

        self.request_pub = self.create_publisher(Request, str(param('sport_request_topic')), 10)
        self.create_subscription(Twist, str(param('cmd_vel_topic')), self.on_cmd_vel, 10)
        self.create_subscription(Response, str(param('sport_response_topic')), self.on_response, 10)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.target = [0.0, 0.0, 0.0]
        self.output = [0.0, 0.0, 0.0]
        self.last_cmd = 0.0
        self.last_tick = time.monotonic()
        self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().warning(
            f'G1 motion bridge ready; enabled={self.enabled}. Enable it only after Nav2 validation.')

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value)) if limit > 0.0 else 0.0

    def on_cmd_vel(self, msg):
        self.target = [
            self.clamp(msg.linear.x, self.limits[0]),
            self.clamp(msg.linear.y, self.limits[1]),
            self.clamp(msg.angular.z, self.limits[2]),
        ]
        self.last_cmd = time.monotonic()

    def on_enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.target = [0.0, 0.0, 0.0]
            self.output = [0.0, 0.0, 0.0]
            self.publish_velocity(self.output)
        response.success = True
        response.message = 'enabled' if self.enabled else 'disabled and stopped'
        return response

    def on_stop(self, _request, response):
        self.enabled = False
        self.target = [0.0, 0.0, 0.0]
        self.output = [0.0, 0.0, 0.0]
        self.publish_velocity(self.output)
        response.success = True
        response.message = 'software stop sent; bridge disabled'
        return response

    def on_response(self, msg):
        if (msg.header.identity.api_id == ROBOT_API_ID_LOCO_SET_VELOCITY and
                msg.header.status.code != 0):
            self.get_logger().error(f'G1 rejected velocity request, status={msg.header.status.code}')

    def publish_velocity(self, velocity):
        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = ROBOT_API_ID_LOCO_SET_VELOCITY
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = False
        request.parameter = json.dumps({'velocity': velocity, 'duration': self.command_duration})
        request.binary = []
        self.request_pub.publish(request)

    def on_timer(self):
        if not self.enabled:
            return
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), 0.1)
        self.last_tick = now
        desired = self.target if now - self.last_cmd <= self.timeout_s else [0.0, 0.0, 0.0]
        for index in range(3):
            delta = self.accel_limits[index] * dt
            error = desired[index] - self.output[index]
            self.output[index] += max(-delta, min(delta, error))
        self.publish_velocity(self.output)

    def shutdown_stop(self):
        self.enabled = False
        self.output = [0.0, 0.0, 0.0]
        for _ in range(5):
            self.publish_velocity(self.output)
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    node = CmdVelToSport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
