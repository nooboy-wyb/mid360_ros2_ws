#!/usr/bin/env python3
"""Publish the saved odometry-aligned raw PCD for RViz inspection."""

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField


DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
    ('tag', '<u2'), ('line', '<u2'),
])


def read_pcd(path):
    with path.open('rb') as stream:
        while stream.readline().strip() != b'DATA binary':
            pass
        return np.frombuffer(stream.read(), dtype=DTYPE).copy()


class RawMapPublisher(Node):
    def __init__(self, path):
        super().__init__('saved_raw_lidar_map')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, '/review/raw_map', qos)
        points = read_pcd(path)
        self.cloud = PointCloud2()
        self.cloud.header.frame_id = 'wheel_odom_debug'
        self.cloud.height = 1
        self.cloud.width = len(points)
        self.cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.cloud.point_step = 16
        self.cloud.row_step = 16 * len(points)
        self.cloud.is_dense = True
        view = np.empty(len(points), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
        for name in ('x', 'y', 'z', 'intensity'):
            view[name] = points[name]
        self.cloud.data = view.tobytes()
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'{path}: loaded {len(points)} raw returns')

    def publish(self):
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.cloud)


def main():
    rclpy.init()
    node = RawMapPublisher(Path(sys.argv[1]))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
