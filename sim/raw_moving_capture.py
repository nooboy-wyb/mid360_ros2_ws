#!/usr/bin/env python3
"""Accumulate raw Gazebo LiDAR returns using Gazebo wheel odometry only."""

import json
import csv
import signal
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
    ('tag', '<u2'), ('line', '<u2'),
])


def write_pcd(path, points):
    header = (
        '# .PCD v0.7 - raw LiDAR scans transformed with Gazebo wheel odometry\n'
        'VERSION 0.7\nFIELDS x y z intensity tag line\n'
        'SIZE 4 4 4 4 2 2\nTYPE F F F F U U\nCOUNT 1 1 1 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA binary\n'
    )
    path.write_bytes(header.encode('ascii') + points.tobytes())


class RawMovingCapture(Node):
    def __init__(self, output_dir, max_range):
        super().__init__('raw_moving_livox_capture')
        self.output_dir = output_dir
        self.max_range = max_range
        self.frames = 0
        self.pose = None
        self.odom_frame = 'odom'
        self.accumulated = []
        self.trajectory_path = output_dir / 'trajectory.csv'
        self.trajectory_stream = self.trajectory_path.open('w', newline='')
        self.trajectory_writer = csv.writer(self.trajectory_stream)
        self.trajectory_writer.writerow([
            'stamp_sec', 'stamp_nanosec', 'x', 'y', 'z',
            'qx', 'qy', 'qz', 'qw', 'linear_x', 'linear_y', 'linear_z',
            'angular_x', 'angular_y', 'angular_z',
        ])
        self.cloud_pub = self.create_publisher(PointCloud2, '/raw_livox/valid_points', 10)
        self.create_subscription(PointCloud2, '/mid360_plugin/points', self.on_cloud, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 30)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ], dtype=np.float64)
        # The plugin's sensor link is fixed at (0, 0, 0.127) in base_link.
        translation = np.array([p.x, p.y, p.z], dtype=np.float64)
        translation += rotation @ np.array([0.0, 0.0, 0.127])
        self.pose = rotation, translation
        self.trajectory_writer.writerow([
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            pose_msg_value(msg, 'linear', 'x'), pose_msg_value(msg, 'linear', 'y'),
            pose_msg_value(msg, 'linear', 'z'), pose_msg_value(msg, 'angular', 'x'),
            pose_msg_value(msg, 'angular', 'y'), pose_msg_value(msg, 'angular', 'z'),
        ])
        self.trajectory_stream.flush()
        if msg.header.frame_id:
            self.odom_frame = msg.header.frame_id

    def on_cloud(self, cloud):
        if self.pose is None:
            return
        raw = np.asarray(list(point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z', 'intensity', 'tag', 'line'), skip_nans=False
        )), dtype=DTYPE)
        xyz = np.column_stack((raw['x'], raw['y'], raw['z']))
        distance = np.linalg.norm(xyz, axis=1)
        valid = raw[np.isfinite(xyz).all(axis=1) & (distance >= 0.2) & (distance < self.max_range)].copy()
        if len(valid):
            rotation, translation = self.pose
            local_xyz = np.column_stack((valid['x'], valid['y'], valid['z'])).astype(np.float64)
            global_xyz = local_xyz @ rotation.T + translation
            valid['x'], valid['y'], valid['z'] = global_xyz.T
            self.accumulated.append(valid)

        msg = PointCloud2()
        msg.header = Header(stamp=cloud.header.stamp, frame_id=self.odom_frame)
        msg.height = 1
        msg.width = len(valid)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = 16 * len(valid)
        msg.is_dense = True
        view = np.empty(len(valid), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
        for name in ('x', 'y', 'z', 'intensity'):
            view[name] = valid[name]
        msg.data = view.tobytes()
        self.cloud_pub.publish(msg)
        self.frames += 1
        if self.frames % 20 == 0:
            self.get_logger().info(f'Captured {self.frames} raw frames; accumulated {sum(map(len, self.accumulated))} returns')

    def save(self):
        if not self.accumulated:
            self.get_logger().error('No odometry-aligned raw returns to save')
            return
        points = np.concatenate(self.accumulated)
        path = self.output_dir / 'raw_accum_valid_odom.pcd'
        write_pcd(path, points)
        metadata = {
            'source': 'raw Gazebo plugin points transformed by Gazebo diff-drive /odom; no FAST-LIO2 or SLAM',
            'odom_frame': self.odom_frame,
            'odom_topic': '/odom',
            'frames': self.frames,
            'valid_points': len(points),
            'max_range_m': self.max_range,
            'sensor_translation_from_base_m': [0.0, 0.0, 0.127],
            'pcd': path.name,
        }
        (self.output_dir / 'raw_accum_valid_odom.json').write_text(json.dumps(metadata, indent=2) + '\n')
        self.trajectory_stream.close()
        self.get_logger().info(f'Saved {len(points)} points to {path}')
        self.get_logger().info(f'Saved odometry trajectory to {self.trajectory_path}')


def pose_msg_value(msg, group, axis):
    return getattr(getattr(msg.twist.twist, group), axis)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--max-range', type=float, default=12.0)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = RawMovingCapture(args.output_dir, args.max_range)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        if not node.trajectory_stream.closed:
            node.trajectory_stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
