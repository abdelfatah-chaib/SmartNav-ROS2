#!/usr/bin/env python3
"""
SmartNav - RViz Alert Visualization Node (v2)
=============================================
Subscribes to /smartnav/alert_level (std_msgs/Int8) and
/smartnav/alert_direction (std_msgs/String).

Publishes visual markers to RViz2:
  /smartnav/canne_marker      - Main cane color indicator
  /smartnav/direction_marker  - Arrow showing obstacle direction
  /smartnav/status_text       - Text overlay showing status

FIX applied vs v1: topic type corrected from Int32 to Int8.

Author: SmartNav Team - Embedded Real-Time Systems, IAOC Master
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class RvizAlertNode(Node):

    def __init__(self) -> None:
        super().__init__('rviz_alert_node')

        # ── Subscribers (FIXED: Int8, not Int32) ────────────────────
        self.create_subscription(Int8,   '/smartnav/alert_level',    self._alert_cb,  10)
        self.create_subscription(String, '/smartnav/alert_direction', self._dir_cb,    10)
        self.create_subscription(String, '/smartnav/obstacle_info',   self._info_cb,   10)

        # ── Publishers ────────────────────────────────────────────────
        self._cane_pub   = self.create_publisher(Marker,      '/smartnav/canne_marker',     10)
        self._arrow_pub  = self.create_publisher(Marker,      '/smartnav/direction_marker', 10)
        self._text_pub   = self.create_publisher(Marker,      '/smartnav/status_text',      10)
        self._array_pub  = self.create_publisher(MarkerArray, '/smartnav/marker_array',     10)

        # ── Blink timer ───────────────────────────────────────────────
        self._timer = self.create_timer(0.1, self._on_timer)  # 10 Hz

        # ── State ─────────────────────────────────────────────────────
        self._alert_level = 0
        self._direction   = 'NONE'
        self._info_text   = 'SAFE'
        self._blink       = True

        self.get_logger().info('RViz Alert Node v2 started (Int8 topic, dual sensor)')

    # ────────────────────────────────────────────────────────────────
    def _alert_cb(self, msg: Int8) -> None:
        self._alert_level = msg.data

    def _dir_cb(self, msg: String) -> None:
        self._direction = msg.data

    def _info_cb(self, msg: String) -> None:
        self._info_text = msg.data

    # ────────────────────────────────────────────────────────────────
    def _on_timer(self) -> None:
        """Publish all markers at 10Hz."""
        self._blink = not self._blink
        now = self.get_clock().now().to_msg()
        markers = []

        # --- Cane marker (cylinder, changes color by alert) ----------
        cane = Marker()
        cane.header.frame_id = 'base_link'
        cane.header.stamp    = now
        cane.ns     = 'cane_alert'
        cane.id     = 0
        cane.type   = Marker.CYLINDER
        cane.action = Marker.ADD
        cane.scale.x = 0.06
        cane.scale.y = 0.06
        cane.scale.z = 1.1
        cane.pose.position.x = 0.28
        cane.pose.position.y = 0.08
        cane.pose.position.z = 0.55
        cane.pose.orientation.w = 1.0

        if self._alert_level == 0:
            # White = safe
            cane.color = ColorRGBA(r=0.95, g=0.95, b=0.95, a=1.0)
        elif self._alert_level == 1:
            # Orange = warning
            cane.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
        else:
            # Red blinking = DANGER
            alpha = 1.0 if self._blink else 0.2
            cane.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=alpha)

        self._cane_pub.publish(cane)
        markers.append(cane)

        # --- Direction arrow marker -----------------------------------
        arrow = Marker()
        arrow.header.frame_id = 'base_link'
        arrow.header.stamp    = now
        arrow.ns     = 'obstacle_dir'
        arrow.id     = 1
        arrow.type   = Marker.ARROW
        arrow.action = Marker.ADD if self._alert_level > 0 else Marker.DELETE
        arrow.scale.x = 0.08
        arrow.scale.y = 0.14
        arrow.scale.z = 0.14
        arrow.pose.position.z = 1.6

        import math
        dir_yaw = {'FRONT': 0.0, 'LEFT': math.pi/2,
                   'RIGHT': -math.pi/2, 'BEHIND': math.pi, 'NONE': 0.0}
        yaw = dir_yaw.get(self._direction, 0.0)
        arrow.pose.orientation.z = math.sin(yaw / 2)
        arrow.pose.orientation.w = math.cos(yaw / 2)

        if self._alert_level == 1:
            arrow.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)
        else:
            arrow.color = ColorRGBA(r=1.0, g=0.0,  b=0.0, a=0.9)

        self._arrow_pub.publish(arrow)
        markers.append(arrow)

        # --- Status text marker ---------------------------------------
        txt = Marker()
        txt.header.frame_id = 'base_link'
        txt.header.stamp    = now
        txt.ns     = 'status_text'
        txt.id     = 2
        txt.type   = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = 0.0
        txt.pose.position.y = 0.0
        txt.pose.position.z = 2.2
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.20
        txt.text = self._info_text

        if self._alert_level == 0:
            txt.color = ColorRGBA(r=0.2, g=0.9, b=0.2, a=1.0)
        elif self._alert_level == 1:
            txt.color = ColorRGBA(r=1.0, g=0.7, b=0.0, a=1.0)
        else:
            txt.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        self._text_pub.publish(txt)
        markers.append(txt)

        # --- Publish as MarkerArray -----------------------------------
        arr = MarkerArray()
        arr.markers = markers
        self._array_pub.publish(arr)


# ──────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
