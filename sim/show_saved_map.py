#!/usr/bin/env python3
"""Publish only the saved FAST-LIO2 PCD in RViz."""

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField


def read_pcd(path):
    with path.open('rb') as stream:
        while True:
            line = stream.readline().decode('ascii').strip()
            if line.startswith('POINTS '):
                count = int(line.split()[1])
            if line == 'DATA binary':
                break
        data = np.frombuffer(stream.read(), dtype='<f4').reshape(count, 8)
    return np.ascontiguousarray(data[:, :3], dtype='<f4')


class SavedMap(Node):
    def __init__(self, path):
        super().__init__('saved_mid360_map_viewer')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, '/review/fastlio2_map', qos)
        points = read_pcd(path)
        self.cloud = PointCloud2()
        self.cloud.header.frame_id = 'odom'
        self.cloud.height = 1
        self.cloud.width = len(points)
        self.cloud.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                             for i, name in enumerate(('x', 'y', 'z'))]
        self.cloud.point_step = 12
        self.cloud.row_step = 12 * len(points)
        self.cloud.is_dense = True
        self.cloud.data = points.tobytes()
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'PCD: {path} ({len(points)} points)')

    def publish(self):
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(self.cloud)


def main():
    rclpy.init()
    node = SavedMap(Path(sys.argv[1]))
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
