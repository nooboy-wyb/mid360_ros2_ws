#!/usr/bin/env python3
"""Bridge a Gazebo 3D scan into a Livox-shaped interface test message.

Point offsets are interpolated by scan column. Gazebo supplies one frame time,
so these offsets are synthetic and cannot validate MID360 motion distortion.
"""

import math

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class ApproxMid360Adapter(Node):
    def __init__(self):
        super().__init__('approx_mid360_adapter')
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.lidar_pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', 20)
        self.create_subscription(
            PointCloud2, '/mid360_sim/points/points', self.on_points, sensor_qos
        )
        self.create_subscription(Imu, '/mid360_sim/imu', self.on_imu, sensor_qos)
        self.frames = 0

    def on_imu(self, imu):
        imu.header.frame_id = 'mid360_imu_link'
        self.imu_pub.publish(imu)

    def on_points(self, cloud):
        msg = CustomMsg()
        msg.header = cloud.header
        msg.header.frame_id = 'mid360_link'
        msg.timebase = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        msg.lidar_id = 0
        msg.rsvd = [0, 0, 0]
        points = []
        columns = max(1, cloud.width - 1)
        for index, (x, y, z, intensity, ring) in enumerate(point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z', 'intensity', 'ring'), skip_nans=False
        )):
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            point = CustomPoint()
            point.offset_time = int((index % cloud.width) * 100_000_000 / columns)
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
            point.reflectivity = max(0, min(255, int(intensity)))
            point.tag = 0
            point.line = int(ring) % 16
            points.append(point)
        msg.points = points
        msg.point_num = len(points)
        self.lidar_pub.publish(msg)
        self.frames += 1
        if self.frames % 30 == 0:
            self.get_logger().info(f'Published {len(points)} points in frame {self.frames}')


def main():
    rclpy.init()
    node = ApproxMid360Adapter()
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
