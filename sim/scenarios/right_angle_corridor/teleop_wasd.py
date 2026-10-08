#!/usr/bin/env python3
"""Small terminal teleop with explicit WASD controls for the manual demo."""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Teleop(Node):
    def __init__(self):
        super().__init__('right_angle_corridor_wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear = 0.0
        self.angular = 0.0
        self.create_timer(0.1, self.publish)

    def publish(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)


def main():
    if not sys.stdin.isatty():
        raise SystemExit('WASD teleop needs an interactive terminal (TTY).')
    rclpy.init()
    node = Teleop()
    old = termios.tcgetattr(sys.stdin.fileno())
    print('W: forward  X: reverse  A/D: turn  S/space: stop  Q: quit', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            ready, _, _ = select.select([sys.stdin], [], [], 0.04)
            if not ready:
                continue
            key = sys.stdin.read(1).lower()
            if key == 'w':
                node.linear, node.angular = 0.18, 0.0
            elif key == 'x':
                node.linear, node.angular = -0.12, 0.0
            elif key == 'a':
                node.linear, node.angular = 0.0, 0.45
            elif key == 'd':
                node.linear, node.angular = 0.0, -0.45
            elif key in ('s', ' '):
                node.linear = node.angular = 0.0
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
        # rclpy's default SIGINT handler can invalidate the context before
        # KeyboardInterrupt reaches this finally block. The parent shell's
        # cleanup publishes a separate zero Twist in that case.
        if rclpy.ok():
            node.linear = node.angular = 0.0
            for _ in range(3):
                node.publish()
                rclpy.spin_once(node, timeout_sec=0.01)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
