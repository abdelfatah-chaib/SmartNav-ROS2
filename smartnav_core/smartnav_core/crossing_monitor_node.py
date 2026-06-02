#!/usr/bin/env python3
"""
Crossing monitor for SmartNav.

Reads Gazebo world poses for the robot and both cars, then computes the
real euclidean distance from the robot to each car independently.  Publishes:
  /smartnav/crossing_alert (Int8): 2 = danger, 1 = attention, 0 = clear

Why distance instead of ETA:
- ETA toward a fixed crossing point was robot-position-agnostic → periodic
  false positives every ~16 s (car reset cycle) regardless of where the robot
  actually is.
- One missing car pose caused the old logic to return False for both cars even
  when the other car was right next to the robot.
- Distance is evaluated per car independently: a missing car is simply skipped.
"""

from __future__ import annotations

import fcntl
import math
import re
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

WORLD = 'smartnav_world'
CAR_MODELS = ('moving_car_A', 'moving_car_B')
ROBOT_MODEL = 'smartnav'
POSE_TOPIC = f'/world/{WORLD}/pose/info'
ALERT_TOPIC = '/smartnav/crossing_alert'

# Default proximity thresholds (m).  The robot starts on the north sidewalk
# (y ≈ 5) and cars drive at y ≈ ±2, so the closest a car can ever be is ~3 m.
# danger_radius_m=3.5 leaves a small margin; warn_radius_m=6.0 gives early
# audio feedback without flooding.
DEFAULT_DANGER_RADIUS_M = 3.5
DEFAULT_WARN_RADIUS_M = 6.0
CHECK_HZ = 2.0
GZ_LOCK_PATH = '/tmp/smartnav_gz_transport.lock'


def _extract_models_xy(
    raw_text: str, model_names: tuple[str, ...]
) -> dict[str, tuple[float, float]]:
    """Extract (x, y) for each requested model from a gz pose/info snapshot."""
    poses: dict[str, tuple[float, float]] = {}
    for name in model_names:
        pattern = (
            rf'name:\s*"{re.escape(name)}".*?'
            r'position\s*\{\s*x:\s*([-+]?\d*\.?\d+)\s*y:\s*([-+]?\d*\.?\d+)'
        )
        match = re.search(pattern, raw_text, flags=re.S)
        if match:
            poses[name] = (float(match.group(1)), float(match.group(2)))
    return poses


def car_alert_level(
    robot_xy: tuple[float, float],
    car_positions: dict[str, tuple[float, float]],
    danger_r: float,
    warn_r: float,
) -> tuple[int, float]:
    """
    Return (alert_level, closest_distance) based on robot-to-car distances.

    Each car is evaluated independently so a missing pose never masks a real
    nearby car.  Returns:
      2  if any car is within danger_r metres
      1  if any car is within warn_r metres
      0  otherwise
    """
    closest = float('inf')
    for xy in car_positions.values():
        dist = math.hypot(xy[0] - robot_xy[0], xy[1] - robot_xy[1])
        if dist < closest:
            closest = dist

    if closest <= danger_r:
        return 2, closest
    if closest <= warn_r:
        return 1, closest
    return 0, closest


class CrossingMonitorNode(Node):
    """Publish danger/attention when a car is close to the robot."""

    def __init__(self) -> None:
        super().__init__('crossing_monitor_node')

        self.declare_parameter('danger_radius_m', DEFAULT_DANGER_RADIUS_M)
        self.declare_parameter('warn_radius_m', DEFAULT_WARN_RADIUS_M)

        self._publisher = self.create_publisher(Int8, ALERT_TOPIC, 10)
        self._prev_alert = -1
        self._timer = self.create_timer(1.0 / CHECK_HZ, self._tick)

        danger_r = self.get_parameter('danger_radius_m').value
        warn_r = self.get_parameter('warn_radius_m').value
        self.get_logger().info(
            f'crossing_monitor_node demarre: seuils danger={danger_r:.1f}m '
            f'attention={warn_r:.1f}m (distance robot<->voiture).'
        )

    def _tick(self) -> None:
        danger_r = float(self.get_parameter('danger_radius_m').value)
        warn_r = float(self.get_parameter('warn_radius_m').value)

        all_poses = self._get_poses(CAR_MODELS + (ROBOT_MODEL,))

        robot_xy = all_poses.get(ROBOT_MODEL)
        if robot_xy is None:
            # Robot pose unavailable — publish clear to avoid spurious alerts.
            self._publish(0, float('inf'))
            return

        car_positions = {k: v for k, v in all_poses.items() if k != ROBOT_MODEL}
        if not car_positions:
            self._publish(0, float('inf'))
            return

        level, closest = car_alert_level(robot_xy, car_positions, danger_r, warn_r)
        self._publish(level, closest)

    def _get_poses(
        self, models: tuple[str, ...]
    ) -> dict[str, tuple[float, float]]:
        try:
            with open(GZ_LOCK_PATH, 'w', encoding='utf-8') as lock_file:
                fcntl.flock(lock_file, fcntl.LOCK_EX)
                result = subprocess.run(
                    ['gz', 'topic', '-e', '-n', '1', '-t', POSE_TOPIC],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=1.0,
                    check=False,
                )
        except subprocess.TimeoutExpired:
            return {}
        if result.returncode != 0 or not result.stdout:
            return {}
        return _extract_models_xy(result.stdout, models)

    def _publish(self, level: int, closest_m: float) -> None:
        msg = Int8()
        msg.data = level
        self._publisher.publish(msg)

        if level == self._prev_alert:
            return

        if level == 2:
            self.get_logger().error(
                f'[VOITURE] DANGER: voiture a {closest_m:.1f}m — ARRET REQUIS !'
            )
        elif level == 1:
            self.get_logger().warn(
                f'[VOITURE] ATTENTION: voiture a {closest_m:.1f}m.'
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
