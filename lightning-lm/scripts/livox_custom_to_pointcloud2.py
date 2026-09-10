#!/usr/bin/env python3
"""Convert Livox CustomMsg to XYZ/intensity PointCloud2 for visualization.

Does not rotate, filter by height, or replace the timestamp. Keep CustomMsg
as the SLAM input: this visualization output omits per-point timing.
"""

import copy
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField


def convert_cloud(msg, point_stride=1, frame_id=''):
    if point_stride < 1:
        raise ValueError('point_stride must be at least 1')
    points = msg.points[::point_stride]
    cloud = PointCloud2()
    cloud.header = copy.deepcopy(msg.header)
    if frame_id:
        cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in [('x', 0), ('y', 4), ('z', 8), ('intensity', 12)]]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_dense = False
    data = bytearray(cloud.row_step)
    for index, point in enumerate(points):
        struct.pack_into('<ffff', data, index * 16,
                         point.x, point.y, point.z, float(point.reflectivity))
    cloud.data = bytes(data)
    return cloud


class LivoxCustomToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pointcloud2')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/points')
        self.declare_parameter('point_stride', 1)
        self.declare_parameter('frame_id', '')
        self.stride = self.get_parameter('point_stride').value
        self.frame_id = self.get_parameter('frame_id').value
        if self.stride < 1:
            raise ValueError('point_stride must be at least 1')
        if self.frame_id:
            self.get_logger().warning(
                'frame_id overrides only the label, not point coordinates. '
                'Use only when the input coordinates already match this frame.')
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 5)
        self.subscription = self.create_subscription(
            CustomMsg, self.get_parameter('input_topic').value,
            self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        self.publisher.publish(convert_cloud(msg, self.stride, self.frame_id))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LivoxCustomToPointCloud2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
