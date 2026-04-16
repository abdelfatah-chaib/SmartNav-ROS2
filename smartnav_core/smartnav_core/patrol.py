#!/usr/bin/env python3
"""Simple autonomous patrol for the SmartNav robot."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PatrolNode(Node):
    """Publish forward velocity commands for a short autonomous run."""

    def __init__(self) -> None:
        super().__init__('smartnav_patrol')

        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.0)
        self.declare_parameter('duration_sec', 8.0)
        self.declare_parameter('publish_rate', 10.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self.finished = False
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            'Patrol started: linear=%.2f m/s, angular=%.2f rad/s, duration=%.2f s'
            % (self.linear_speed, self.angular_speed, self.duration_sec)
        )

    def _on_timer(self) -> None:
        if self.finished:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self.duration_sec > 0.0 and elapsed >= self.duration_sec:
            self._stop_robot()
            self.finished = True
            self.timer.cancel()
            self.get_logger().info('Patrol finished, robot stopped.')
            return

        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)

    def _stop_robot(self) -> None:
        self.publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = PatrolNode()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._stop_robot()
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
