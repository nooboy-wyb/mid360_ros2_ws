#!/usr/bin/env python3
"""Drive a repeatable straight-left-90-straight route using Gazebo truth."""
import csv
import json
import math
import sys
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

ROBOT = 'turtlebot3_waffle_pi'
START_TARGET = (0.5, -2.0)
END_TARGET = (0.5, 2.0)
YAW_STRAIGHT = 0.0
YAW_LEFT = math.pi / 2


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class RouteController(Node):
    def __init__(self, output_dir: Path):
        super().__init__('right_angle_corridor_route_controller')
        self.output_dir = output_dir
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(ModelStates, '/ground_truth/model_states', self.on_models, 10)
        self.create_subscription(Clock, '/clock', self.on_clock,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.sim_time = None
        self.pose = None
        self.phase = 'WAIT'
        self.phase_started = None
        self.stable_since = None
        self.done = False
        self.failed = False
        self.route_path = output_dir / 'route_trace.csv'
        self.route_file = self.route_path.open('w', newline='')
        self.route_csv = csv.writer(self.route_file)
        self.route_csv.writerow(['sim_time_s', 'phase', 'x', 'y', 'yaw', 'cmd_linear_x', 'cmd_angular_z'])
        self.event_path = output_dir / 'route_events.csv'
        self.event_file = self.event_path.open('w', newline='')
        self.event_csv = csv.writer(self.event_file)
        self.event_csv.writerow(['phase', 'start_sim_s', 'end_sim_s', 'start_x', 'start_y', 'start_yaw', 'end_x', 'end_y', 'end_yaw'])
        self.phase_pose = None
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Route: 4 m straight, stop, left 90 degrees, stop, 4 m straight, stop')

    def on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_models(self, msg):
        try:
            i = msg.name.index(ROBOT)
        except ValueError:
            return
        p = msg.pose[i].position
        q = msg.pose[i].orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q))

    def enter(self, phase):
        if self.phase not in ('WAIT', 'DONE', 'FAILED') and self.phase_started is not None:
            x, y, yaw = self.phase_pose or (float('nan'),) * 3
            ex, ey, eyaw = self.pose or (float('nan'),) * 3
            self.event_csv.writerow([self.phase, f'{self.phase_started:.6f}', f'{self.sim_time:.6f}',
                                     f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}',
                                     f'{ex:.6f}', f'{ey:.6f}', f'{eyaw:.6f}'])
            self.event_file.flush()
        self.phase = phase
        self.phase_started = self.sim_time
        self.phase_pose = self.pose
        self.stable_since = None
        self.get_logger().info(f'Route phase: {phase} at sim {self.sim_time:.3f}s')

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)
        if self.sim_time is not None and self.pose is not None:
            self.route_csv.writerow([f'{self.sim_time:.6f}', self.phase,
                                     f'{self.pose[0]:.6f}', f'{self.pose[1]:.6f}', f'{self.pose[2]:.6f}',
                                     f'{linear:.5f}', f'{angular:.5f}'])
            self.route_file.flush()

    def follow_segment(self, target, target_yaw):
        x, y, yaw = self.pose
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        yaw_err = angle_error(target_yaw, yaw)
        lateral_err = -math.sin(yaw) * dx + math.cos(yaw) * dy
        angular = clamp(2.2 * yaw_err + 0.7 * lateral_err, -0.45, 0.45)
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        linear = clamp(0.55 * max(0.0, forward), 0.035, 0.18) if distance > 0.07 else 0.0
        return distance, linear, angular

    def tick(self):
        if self.done or self.failed:
            self.publish()
            return
        if self.sim_time is None or self.pose is None:
            self.publish()
            return
        if self.phase == 'WAIT':
            if self.sim_time >= 0.5:
                self.enter('STRAIGHT_1')
            self.publish()
            return

        elapsed = self.sim_time - self.phase_started
        if self.phase == 'STRAIGHT_1':
            distance, linear, angular = self.follow_segment(START_TARGET, YAW_STRAIGHT)
            self.publish(linear, angular)
            if distance < 0.07:
                self.enter('PAUSE_AT_CORNER')
            elif elapsed > 40:
                self.fail('Timed out reaching the corner')
        elif self.phase == 'PAUSE_AT_CORNER':
            self.publish()
            if elapsed >= 1.0:
                self.enter('TURN_LEFT_90')
        elif self.phase == 'TURN_LEFT_90':
            err = angle_error(YAW_LEFT, self.pose[2])
            angular = clamp(1.8 * err, -0.42, 0.42)
            if abs(err) > 0.015:
                angular = math.copysign(max(0.07, abs(angular)), angular)
                self.stable_since = None
            else:
                angular = 0.0
                if self.stable_since is None:
                    self.stable_since = self.sim_time
                elif self.sim_time - self.stable_since >= 0.5:
                    self.publish()
                    self.enter('PAUSE_AFTER_TURN')
                    return
            self.publish(0.0, angular)
            if elapsed > 12:
                self.fail('Timed out turning left 90 degrees')
        elif self.phase == 'PAUSE_AFTER_TURN':
            self.publish()
            if elapsed >= 1.0:
                self.enter('STRAIGHT_2')
        elif self.phase == 'STRAIGHT_2':
            distance, linear, angular = self.follow_segment(END_TARGET, YAW_LEFT)
            self.publish(linear, angular)
            if distance < 0.07:
                self.enter('FINISH_STOP')
            elif elapsed > 40:
                self.fail('Timed out reaching the end point')
        elif self.phase == 'FINISH_STOP':
            self.publish()
            if elapsed >= 2.0:
                self.enter('DONE')
                self.done = True
                (self.output_dir / 'route_status.json').write_text(json.dumps({
                    'status': 'complete', 'sim_time_s': self.sim_time,
                    'final_pose': {'x': self.pose[0], 'y': self.pose[1], 'yaw_rad': self.pose[2]},
                }, indent=2) + '\n')
                self.get_logger().info('Route complete; stopping controller')

    def fail(self, reason):
        self.publish()
        self.enter('FAILED')
        self.failed = True
        (self.output_dir / 'route_status.json').write_text(json.dumps({
            'status': 'failed', 'reason': reason, 'sim_time_s': self.sim_time,
            'pose': {'x': self.pose[0], 'y': self.pose[1], 'yaw_rad': self.pose[2]},
        }, indent=2) + '\n')
        self.get_logger().error(reason)


def main():
    output_dir = Path(sys.argv[1]).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = RouteController(output_dir)
    try:
        while rclpy.ok() and not node.done and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish()
        node.route_file.close()
        node.event_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
