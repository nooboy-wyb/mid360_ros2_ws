#!/usr/bin/env python3
"""Record plugin frames without CustomMsg conversion, IMU fusion, or SLAM."""

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
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
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity tag line\n'
        'SIZE 4 4 4 4 2 2\n'
        'TYPE F F F F U U\n'
        'COUNT 1 1 1 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA binary\n'
    )
    with path.open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(points.tobytes())


class RawCapture(Node):
    def __init__(self, output_dir, save_frames, warmup_frames, max_range):
        super().__init__('raw_livox_capture')
        self.output_dir = output_dir
        self.save_frames = save_frames
        self.warmup_frames = warmup_frames
        self.max_range = max_range
        self.frames = 0
        self.publisher = self.create_publisher(PointCloud2, '/raw_livox/valid_points', 10)
        self.create_subscription(PointCloud2, '/mid360_plugin/points', self.on_cloud, 10)

    def on_cloud(self, cloud):
        values = list(point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z', 'intensity', 'tag', 'line'), skip_nans=False
        ))
        points = np.array(values, dtype=DTYPE)
        csv_values = list(point_cloud2.read_points(
            cloud, field_names=('csv_time_raw', 'scan_sequence'), skip_nans=False
        ))
        distance = np.sqrt(points['x'] ** 2 + points['y'] ** 2 + points['z'] ** 2)
        finite = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
        valid = points[finite & (distance >= 0.2) & (distance < self.max_range)]

        # For RViz, only remove zero / invalid returns; XYZ and intensity are unchanged.
        display = PointCloud2()
        display.header = Header(stamp=cloud.header.stamp, frame_id='raw_lidar')
        display.height = 1
        display.width = len(valid)
        display.fields = [
            PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(('x', 'y', 'z', 'intensity'))
        ]
        display.point_step = 16
        display.row_step = 16 * len(valid)
        display.is_dense = True
        packed = np.empty(len(valid), dtype=[(name, '<f4') for name in ('x', 'y', 'z', 'intensity')])
        for name in ('x', 'y', 'z'):
            packed[name] = valid[name]
        packed['intensity'] = valid['intensity']
        display.data = packed.tobytes()
        self.publisher.publish(display)

        self.frames += 1
        saved_index = self.frames - self.warmup_frames
        if 1 <= saved_index <= self.save_frames:
            number = f'{saved_index:03d}'
            write_pcd(self.output_dir / f'raw_full_{number}.pcd', points)
            write_pcd(self.output_dir / f'raw_valid_{number}.pcd', valid)
            info = {
                'plugin_topic': '/mid360_plugin/points',
                'source_frame_index': self.frames,
                'plugin_scan_sequence': int(csv_values[0][1]),
                'csv_time_raw_first': float(csv_values[0][0]),
                'csv_time_raw_last': float(csv_values[-1][0]),
                'csv_time_unit': 'unverified; dataset column is labeled Time/s but increments by 1 per row',
                'original_frame_id': cloud.header.frame_id,
                'stamp_sec': cloud.header.stamp.sec,
                'stamp_nanosec': cloud.header.stamp.nanosec,
                'full_points': len(points),
                'valid_returns': len(valid),
                'display_max_range_m': self.max_range,
                'excluded_no_return_or_invalid': len(points) - len(valid),
                'note': 'Single plugin scan; no temporal accumulation, deskew, odometry or FAST-LIO2',
            }
            (self.output_dir / f'raw_{number}.json').write_text(json.dumps(info, indent=2) + '\n')
            self.get_logger().info(f'Saved source frame {self.frames}: {len(points)} points, {len(valid)} valid')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--save-frames', type=int, default=3)
    parser.add_argument('--warmup-frames', type=int, default=20)
    parser.add_argument('--max-range', type=float, default=12.0)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = RawCapture(args.output_dir, args.save_frames, args.warmup_frames, args.max_range)
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
