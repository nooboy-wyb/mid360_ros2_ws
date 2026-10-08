#!/usr/bin/env python3
"""Adapt Classic Livox plugin PointCloud2 to FAST-LIO2's Livox input."""

import math
import os

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class Adapter(Node):
    def __init__(self):
        super().__init__('plugin_mid360_adapter')
        self.lidar_pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', 20)
        self.create_subscription(PointCloud2, '/mid360_plugin/points', self.on_cloud, 10)
        self.create_subscription(Imu, '/mid360_sim/imu', self.on_imu, 20)
        self.frames = 0
        self.drop_first_scan = os.environ.get('MID360_DROP_FIRST_SCAN', '0') == '1'
        # Timing-sweep diagnostic: shift the LiDAR frame clock relative to the
        # unchanged IMU stream while preserving each point's scan offset.
        self.lidar_stamp_shift_ns = int(os.environ.get('MID360_LIDAR_STAMP_SHIFT_NS', '0'))

    def on_imu(self, msg):
        msg.header.frame_id = 'mid360_imu_link'
        self.imu_pub.publish(msg)

    def on_cloud(self, cloud):
        if self.drop_first_scan:
            self.drop_first_scan = False
            self.get_logger().info('Skipping first LiDAR scan as requested; it has incomplete IMU coverage')
            return
        msg = CustomMsg()
        msg.header = cloud.header
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000 +
                    msg.header.stamp.nanosec + self.lidar_stamp_shift_ns)
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = 'mid360_link'
        msg.timebase = stamp_ns
        msg.lidar_id = 0
        msg.rsvd = [0, 0, 0]
        points = []
        has_offsets = any(field.name == 'offset_time' for field in cloud.fields)
        fields = ('x', 'y', 'z', 'intensity', 'tag', 'line', 'offset_time') if has_offsets else (
            'x', 'y', 'z', 'intensity', 'tag', 'line')
        for index, values in enumerate(point_cloud2.read_points(
                cloud, field_names=fields, skip_nans=False)):
            # Humble returns a structured NumPy array; each iterated row is a
            # NumPy scalar rather than a plain tuple.
            if hasattr(values, 'tolist'):
                values = values.tolist()
            x, y, z, intensity, tag, line = values[:6]
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            point = CustomPoint()
            # New plugin scans carry the pose-consistent firing offset. Keep a
            # time-order estimate only for previously recorded pre-fix bags.
            point.offset_time = (int(values[6]) if has_offsets else
                                 index * 100_000_000 // max(1, cloud.width - 1))
            point.x, point.y, point.z = float(x), float(y), float(z)
            point.reflectivity = max(0, min(255, int(intensity))) if math.isfinite(intensity) else 0
            point.tag = int(tag) & 255
            point.line = int(line) & 255
            points.append(point)
        msg.points = points
        msg.point_num = len(points)
        self.lidar_pub.publish(msg)
        self.frames += 1
        if self.frames % 10 == 0:
            self.get_logger().info(f'Converted {self.frames} plugin frames, latest {len(points)} points')


def main():
    rclpy.init()
    node = Adapter()
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
