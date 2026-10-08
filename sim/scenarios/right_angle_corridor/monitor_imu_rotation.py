#!/usr/bin/env python3
"""Live IMU and ground-truth monitor for an in-place rotation check."""
import argparse
from collections import deque
import math
import time

import rclpy
from gazebo_msgs.msg import LinkStates
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class RotationMonitor(Node):
    def __init__(self, window_s, link_name):
        super().__init__('mid360_imu_rotation_monitor')
        self.window_s = window_s
        self.link_name = link_name
        self.sim_now = None
        self.imu = {'/mid360_sim/imu': deque(), '/livox/imu': deque()}
        self.truth = deque()
        self.last_yaw = None
        self.unwrapped_yaw = 0.0
        # Gazebo's /clock is commonly best-effort; a default reliable
        # subscription cannot match that publisher and would wait forever.
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self.on_clock, clock_qos)
        self.create_subscription(LinkStates, '/ground_truth/link_states', self.on_truth, 20)
        for topic in self.imu:
            self.create_subscription(Imu, topic, lambda msg, t=topic: self.on_imu(t, msg), 50)
        self.create_timer(1.0, self.report)
        self.last_report_wall = time.monotonic()

    def on_clock(self, msg):
        self.sim_now = stamp_seconds(msg.clock)

    def on_imu(self, topic, msg):
        t = stamp_seconds(msg.header.stamp)
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.imu[topic].append((t, a.x, a.y, a.z, g.x, g.y, g.z))

    def on_truth(self, msg):
        if self.sim_now is None:
            return
        try:
            i = msg.name.index(self.link_name)
        except ValueError:
            return
        pose, twist = msg.pose[i], msg.twist[i]
        yaw = quat_yaw(pose.orientation)
        if self.last_yaw is None:
            self.unwrapped_yaw = yaw
        else:
            self.unwrapped_yaw += wrap(yaw - self.last_yaw)
        self.last_yaw = yaw
        self.truth.append((self.sim_now, self.unwrapped_yaw,
                           pose.position.x, pose.position.y, pose.position.z,
                           twist.linear.x, twist.linear.y, twist.linear.z))

    def report(self):
        if self.sim_now is None:
            print('Waiting for /clock ...', flush=True)
            return
        cutoff = self.sim_now - self.window_s
        for samples in self.imu.values():
            while samples and samples[0][0] < cutoff:
                samples.popleft()
        while self.truth and self.truth[0][0] < cutoff:
            self.truth.popleft()
        print(f'\n[sim {self.sim_now:.2f}s | last {self.window_s:.1f}s]', flush=True)
        for topic, samples in self.imu.items():
            if len(samples) < 2:
                print(f'{topic}: waiting ({len(samples)} samples)', flush=True)
                continue
            rows = list(samples)
            dt = [b[0] - a[0] for a, b in zip(rows, rows[1:]) if b[0] > a[0]]
            acc = [row[1:4] for row in rows]
            gyro = [row[4:7] for row in rows]
            norms = [math.sqrt(sum(v * v for v in axis)) for axis in acc]
            amean = [sum(row[k] for row in rows) / len(rows) for k in (1, 2, 3)]
            gmean = [sum(row[k] for row in rows) / len(rows) for k in (4, 5, 6)]
            astdev = [math.sqrt(sum((row[k] - amean[k - 1]) ** 2 for row in rows) / len(rows)) for k in (1, 2, 3)]
            gstd = [math.sqrt(sum((row[k] - gmean[k - 4]) ** 2 for row in rows) / len(rows)) for k in (4, 5, 6)]
            print(f'{topic}: n={len(rows)}, rate={1/(sum(dt)/len(dt)):.1f}Hz, dt_p95={sorted(dt)[int(.95*(len(dt)-1))]*1000:.2f}ms', flush=True)
            print(f'  acc mean xyz={tuple(round(x,4) for x in amean)} std={tuple(round(x,4) for x in astdev)} |norm| mean/std/min/max={sum(norms)/len(norms):.4f}/{(sum((x-sum(norms)/len(norms))**2 for x in norms)/len(norms))**.5:.4f}/{min(norms):.4f}/{max(norms):.4f}', flush=True)
            print(f'  gyro mean xyz={tuple(round(x,5) for x in gmean)} std={tuple(round(x,5) for x in gstd)} rad/s', flush=True)
        if len(self.truth) >= 2:
            first, last = self.truth[0], self.truth[-1]
            yaw_deg = math.degrees(last[1] - first[1])
            dx, dy, dz = last[2] - first[2], last[3] - first[3], last[4] - first[4]
            pos_speed = math.sqrt(dx*dx + dy*dy + dz*dz) / max(last[0] - first[0], 1e-9)
            twist_speed = math.sqrt(sum((sum(row[k] for row in self.truth)/len(self.truth))**2 for k in (5, 6, 7)))
            print(f'GT {self.link_name}: yaw change={yaw_deg:+.2f}deg, position-derived speed={pos_speed:.4f}m/s, mean |twist.linear|={twist_speed:.4f}m/s', flush=True)
            for topic, samples in self.imu.items():
                rows = [row for row in samples if len(samples) and first[0] <= row[0] <= last[0]]
                if len(rows) >= 2:
                    integrated = sum((b[6] + a[6]) * 0.5 * (b[0] - a[0]) for a, b in zip(rows, rows[1:]))
                    print(f'  {topic} gyro-z integral={math.degrees(integrated):+.2f}deg (compare sign/magnitude with GT yaw)', flush=True)
        self.last_report_wall = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=float, default=5.0, help='rolling window in simulation seconds')
    parser.add_argument('--link', default='turtlebot3_waffle_pi::mid360_mount')
    args = parser.parse_args()
    rclpy.init()
    node = RotationMonitor(args.window, args.link)
    try:
        print('Monitoring IMU streams and Gazebo truth. Start with a few seconds stopped, rotate in place, then stop.', flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
