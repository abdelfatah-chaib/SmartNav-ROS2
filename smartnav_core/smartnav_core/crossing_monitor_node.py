#!/usr/bin/env python3
"""
Crossing monitor — version subscriber ROS 2 (sans subprocess gz).

Utilise /model/smartnav/pose et /model/moving_car_A(B)/pose
via ros_gz_bridge pour obtenir les poses en temps réel.
"""

from __future__ import annotations
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

ALERT_TOPIC = '/smartnav/crossing_alert'
DEFAULT_DANGER_RADIUS_M = 3.5
DEFAULT_WARN_RADIUS_M = 6.0


class CrossingMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('crossing_monitor_node')

        self.declare_parameter('danger_radius_m', DEFAULT_DANGER_RADIUS_M)
        self.declare_parameter('warn_radius_m', DEFAULT_WARN_RADIUS_M)

        self._robot_xy: tuple[float, float] | None = None
        self._car_positions: dict[str, tuple[float, float]] = {}

        self.create_subscription(
            Pose, '/model/smartnav/pose', self._on_robot_pose, 10
        )
        for car in ('moving_car_A', 'moving_car_B'):
            self.create_subscription(
                Pose,
                f'/model/{car}/pose',
                lambda msg, c=car: self._on_car_pose(msg, c),
                10,
            )

        self._publisher = self.create_publisher(Int8, ALERT_TOPIC, 10)
        self._prev_alert = -1
        self.create_timer(0.1, self._tick)

        self.get_logger().info('crossing_monitor_node démarré — mode subscriber ROS 2.')

    def _on_robot_pose(self, msg: Pose) -> None:
        self._robot_xy = (msg.position.x, msg.position.y)

    def _on_car_pose(self, msg: Pose, car_name: str) -> None:
        self._car_positions[car_name] = (msg.position.x, msg.position.y)

    def _tick(self) -> None:
        danger_r = float(self.get_parameter('danger_radius_m').value)
        warn_r = float(self.get_parameter('warn_radius_m').value)

        if self._robot_xy is None or not self._car_positions:
            self._publish(0, float('inf'))
            return

        closest = min(
            math.hypot(cx - self._robot_xy[0], cy - self._robot_xy[1])
            for cx, cy in self._car_positions.values()
        )

        if closest <= danger_r:
            level = 2
        elif closest <= warn_r:
            level = 1
        else:
            level = 0

        self._publish(level, closest)

    def _publish(self, level: int, closest_m: float) -> None:
        msg = Int8()
        msg.data = level
        self._publisher.publish(msg)

        if level == self._prev_alert:
            return
        if level == 2:
            self.get_logger().error(
                f'[VOITURE] DANGER: voiture à {closest_m:.1f}m — ARRÊT !'
            )
        elif level == 1:
            self.get_logger().warn(
                f'[VOITURE] ATTENTION: voiture à {closest_m:.1f}m.'
            )
        else:
            self.get_logger().info('[VOITURE] Voie libre.')
        self._prev_alert = level


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrossingMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
