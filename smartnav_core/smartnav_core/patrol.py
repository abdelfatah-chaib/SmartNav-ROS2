#!/usr/bin/env python3
"""
SmartNav - Patrol Node (v2 - Obstacle Aware)
=============================================
Autonomous patrol that reacts to obstacle alerts.

Subscribes:
  /smartnav/alert_level     (std_msgs/Int8)    - 0=SAFE 1=WARNING 2=DANGER
  /smartnav/alert_direction (std_msgs/String)  - FRONT/LEFT/RIGHT/BEHIND

Publishes:
  /cmd_vel (geometry_msgs/Twist)

Behavior:
  SAFE    : Walk forward at normal_speed
  WARNING : Slow down, slight turn away from obstacle direction
  DANGER  : Stop completely (emergency stop already from obstacle_detector)

Parameters:
  linear_speed   : Normal forward speed [m/s] (default 0.30)
  warning_speed  : Reduced speed on WARNING  (default 0.12)
  angular_speed  : Base angular speed for avoidance [rad/s] (default 0.35)
  duration_sec   : 0 = infinite patrol; > 0 = stop after N seconds
  publish_rate   : Control loop frequency [Hz] (default 10)

Author: SmartNav Team - Embedded Real-Time Systems, IAOC Master
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, String


class PatrolNode(Node):
    """Obstacle-aware autonomous patrol."""

    def __init__(self) -> None:
        super().__init__('smartnav_patrol')

        self.declare_parameter('linear_speed',   0.30)
        self.declare_parameter('warning_speed',  0.12)
        self.declare_parameter('angular_speed',  0.35)
        self.declare_parameter('duration_sec',   0.0)
        self.declare_parameter('publish_rate',   10.0)

        self._lin_speed  = self.get_parameter('linear_speed').value
        self._warn_speed = self.get_parameter('warning_speed').value
        self._ang_speed  = self.get_parameter('angular_speed').value
        self._duration   = self.get_parameter('duration_sec').value
        rate             = max(self.get_parameter('publish_rate').value, 1.0)

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(Int8,   '/smartnav/alert_level',    self._alert_cb, 10)
        self.create_subscription(String, '/smartnav/alert_direction', self._dir_cb,   10)

        # ── Publisher ────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── State ─────────────────────────────────────────────────────
        self._alert     = 0
        self._direction = 'NONE'
        self._finished  = False
        self._start     = self.get_clock().now()

        self._timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'PatrolNode v2 started | speed={self._lin_speed}m/s '
            f'warn_speed={self._warn_speed}m/s '
            f'duration={"inf" if self._duration <= 0 else str(self._duration)+"s"}'
        )

    # ────────────────────────────────────────────────────────────────
    def _alert_cb(self, msg: Int8) -> None:
        self._alert = msg.data

    def _dir_cb(self, msg: String) -> None:
        self._direction = msg.data

    # ────────────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        """Main control loop - called at publish_rate Hz."""
        if self._finished:
            return

        # Check duration
        if self._duration > 0:
            elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
            if elapsed >= self._duration:
                self._stop()
                self._finished = True
                self._timer.cancel()
                self.get_logger().info('Patrol duration reached, stopping.')
                return

        cmd = Twist()

        if self._alert == 2:
            # DANGER: Full stop (also handled by obstacle_detector emergency stop)
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(
                f'[PATROL] DANGER - stopped | dir={self._direction}',
                throttle_duration_sec=1.0
            )

        elif self._alert == 1:
            # WARNING: Slow down and steer away from obstacle
            cmd.linear.x = self._warn_speed

            if self._direction == 'FRONT':
                # Obstacle in front: turn to find clear path
                cmd.angular.z = self._ang_speed
            elif self._direction == 'LEFT':
                # Obstacle on left: turn right
                cmd.angular.z = -self._ang_speed * 0.5
            elif self._direction == 'RIGHT':
                # Obstacle on right: turn left
                cmd.angular.z = self._ang_speed * 0.5
            else:
                cmd.angular.z = 0.0

        else:
            # SAFE: Walk forward normally
            cmd.linear.x  = self._lin_speed
            cmd.angular.z = 0.0

        self._pub.publish(cmd)

    # ────────────────────────────────────────────────────────────────
    def _stop(self) -> None:
        self._pub.publish(Twist())


# ──────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._stop()
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
