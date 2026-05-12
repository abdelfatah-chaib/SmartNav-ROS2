#!/usr/bin/env python3
"""
SmartNav - Obstacle Detector Node (v2)
=======================================
Dual-sensor obstacle detection for the smart cane system.

Subscribes:
  /scan       (sensor_msgs/LaserScan) - Cane tip LiDAR (ground-level)
  /scan_chest (sensor_msgs/LaserScan) - Chest LiDAR (head/body-level)

Publishes:
  /smartnav/alert_level     (std_msgs/Int8)    - 0=SAFE 1=WARNING 2=DANGER
  /smartnav/alert_direction (std_msgs/String)  - FRONT/LEFT/RIGHT/BEHIND
  /smartnav/obstacle_info   (std_msgs/String)  - Full status text
  /cmd_vel                  (geometry_msgs/Twist) - Emergency stop on DANGER

Real-time constraint: < 100ms from sensor input to alert output.
QoS: BEST_EFFORT matches LiDAR publisher.

Detection zones (cane scan, horizontal plane):
  FRONT  : -60 to +60 deg   (main danger zone)
  LEFT   : +60 to +120 deg
  RIGHT  : -120 to -60 deg
  BEHIND : rest

Author: SmartNav Team - Embedded Real-Time Systems, IAOC Master
"""

from __future__ import annotations
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, String


class ObstacleDetector(Node):
    """Dual-sensor obstacle detector for SmartNav smart cane."""

    def __init__(self) -> None:
        super().__init__('obstacle_detector')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('danger_dist_cane',   0.50)
        self.declare_parameter('warning_dist_cane',  1.20)
        self.declare_parameter('danger_dist_chest',  0.80)
        self.declare_parameter('warning_dist_chest', 2.00)
        self.declare_parameter('front_half_angle',  60.0)
        self.declare_parameter('emergency_stop',    True)

        self._danger_cane  = self.get_parameter('danger_dist_cane').value
        self._warn_cane    = self.get_parameter('warning_dist_cane').value
        self._danger_chest = self.get_parameter('danger_dist_chest').value
        self._warn_chest   = self.get_parameter('warning_dist_chest').value
        self._front_rad    = math.radians(self.get_parameter('front_half_angle').value)
        self._do_estop     = self.get_parameter('emergency_stop').value

        # ── QoS: BEST_EFFORT matches LiDAR ──────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',       self._cane_cb,  qos)
        self.create_subscription(LaserScan, '/scan_chest', self._chest_cb, qos)

        # ── Publishers ────────────────────────────────────────────────
        self._alert_pub = self.create_publisher(Int8,   '/smartnav/alert_level',     10)
        self._dir_pub   = self.create_publisher(String, '/smartnav/alert_direction',  10)
        self._info_pub  = self.create_publisher(String, '/smartnav/obstacle_info',    10)
        self._vel_pub   = self.create_publisher(Twist,  '/cmd_vel',                  10)

        # ── State ─────────────────────────────────────────────────────
        self._last_alert      = -1
        self._chest_alert     = 0
        self._cane_min        = float('inf')
        self._chest_min       = float('inf')
        self._estop_active    = False

        self.get_logger().info(
            f'ObstacleDetector v2 started | '
            f'cane DANGER<{self._danger_cane}m WARNING<{self._warn_cane}m | '
            f'chest DANGER<{self._danger_chest}m | '
            f'front cone +-{math.degrees(self._front_rad):.0f}deg'
        )

    # ────────────────────────────────────────────────────────────────
    def _cane_cb(self, msg: LaserScan) -> None:
        """Process cane LiDAR - main RT path."""
        zones = self._parse_scan(msg)
        self._cane_min = min(zones.values()) if zones else float('inf')
        direction = min(zones, key=lambda z: zones[z]) if zones else 'NONE'

        d = self._cane_min
        if d < self._danger_cane:
            cane_alert = 2
        elif d < self._warn_cane:
            cane_alert = 1
        else:
            cane_alert = 0

        merged = max(cane_alert, self._chest_alert)
        self._publish(merged, direction, d)

    # ────────────────────────────────────────────────────────────────
    def _chest_cb(self, msg: LaserScan) -> None:
        """Process chest LiDAR - detects head/body-height obstacles."""
        zones = self._parse_scan(msg)
        self._chest_min = min(zones.values()) if zones else float('inf')
        d = self._chest_min

        if d < self._danger_chest:
            self._chest_alert = 2
            self.get_logger().warn(f'[CHEST SENSOR] Head-height obstacle at {d:.2f}m')
        elif d < self._warn_chest:
            self._chest_alert = 1
        else:
            self._chest_alert = 0

    # ────────────────────────────────────────────────────────────────
    def _parse_scan(self, msg: LaserScan) -> dict:
        """Split scan into directional zones {FRONT, LEFT, RIGHT, BEHIND}."""
        zones: dict[str, float] = {}
        angle = msg.angle_min

        for dist in msg.ranges:
            if not math.isfinite(dist) or dist < msg.range_min or dist > msg.range_max:
                angle += msg.angle_increment
                continue

            a = ((angle + math.pi) % (2 * math.pi)) - math.pi
            abs_a = abs(a)

            if abs_a <= self._front_rad:
                z = 'FRONT'
            elif abs_a >= (math.pi - self._front_rad):
                z = 'BEHIND'
            elif a > 0:
                z = 'LEFT'
            else:
                z = 'RIGHT'

            if z not in zones or dist < zones[z]:
                zones[z] = dist

            angle += msg.angle_increment

        return zones

    # ────────────────────────────────────────────────────────────────
    def _publish(self, level: int, direction: str, dist: float) -> None:
        """Publish alert topics and trigger emergency stop if needed."""
        # Int8 alert level
        a = Int8()
        a.data = level
        self._alert_pub.publish(a)

        # Direction string
        d = String()
        d.data = direction
        self._dir_pub.publish(d)

        # Full info
        names = {0: 'SAFE', 1: 'WARNING', 2: 'DANGER'}
        info = String()
        info.data = (
            f'{names[level]} | {direction} | '
            f'cane={self._cane_min:.2f}m | chest={self._chest_min:.2f}m'
        )
        self._info_pub.publish(info)

        # Log on state change
        if level != self._last_alert:
            if level == 2:
                self.get_logger().error(f'[DANGER]  {direction} dist={dist:.2f}m')
            elif level == 1:
                self.get_logger().warn(f'[WARNING] {direction} dist={dist:.2f}m')
            else:
                self.get_logger().info(f'[SAFE]    {direction} dist={dist:.2f}m')
            self._last_alert = level

        # Emergency stop on DANGER
        if level == 2 and self._do_estop:
            if not self._estop_active:
                self._vel_pub.publish(Twist())  # zero velocity
                self.get_logger().error(
                    f'EMERGENCY STOP | cane={self._cane_min:.2f}m '
                    f'chest={self._chest_min:.2f}m'
                )
                self._estop_active = True
        else:
            self._estop_active = False


# ──────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
