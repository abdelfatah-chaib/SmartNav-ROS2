#!/usr/bin/env python3
"""
SmartNav - Haptic Feedback Simulation Node
==========================================
Simulates the haptic vibration motor system described in the project proposal.
In a real embedded deployment, this node would communicate with
a microcontroller via serial/GPIO to trigger physical vibrations.

In simulation: logs vibration commands and publishes haptic patterns
to /smartnav/haptic_pattern topic for monitoring.

Subscribes:
  /smartnav/alert_level     (std_msgs/Int8)   - 0/1/2
  /smartnav/alert_direction (std_msgs/String) - FRONT/LEFT/RIGHT/BEHIND

Publishes:
  /smartnav/haptic_pattern (std_msgs/String) - Human-readable haptic command

Haptic patterns:
  SAFE       -> silence (no vibration)
  WARNING FRONT  -> 2 short pulses, 100ms each, 200ms gap
  WARNING LEFT   -> 2 pulses left motor
  WARNING RIGHT  -> 2 pulses right motor
  DANGER FRONT   -> continuous rapid vibration (panic pattern)
  DANGER *       -> all motors rapid

Timing specification (hard real-time requirement from proposal):
  Sensor detect -> alert published -> haptic triggered: < 100ms total

Author: SmartNav Team - Embedded Real-Time Systems, IAOC Master
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String


# Haptic pattern table:
# (pulse_count, pulse_ms, gap_ms, motor_side, intensity)
HAPTIC_PATTERNS: dict[tuple[int, str], dict] = {
    (0, 'NONE'):    {'desc': 'silence',            'pulses': 0,  'ms': 0,   'gap': 0,   'side': 'none'},
    (0, 'FRONT'):   {'desc': 'silence',            'pulses': 0,  'ms': 0,   'gap': 0,   'side': 'none'},
    (0, 'LEFT'):    {'desc': 'silence',            'pulses': 0,  'ms': 0,   'gap': 0,   'side': 'none'},
    (0, 'RIGHT'):   {'desc': 'silence',            'pulses': 0,  'ms': 0,   'gap': 0,   'side': 'none'},
    (0, 'BEHIND'):  {'desc': 'silence',            'pulses': 0,  'ms': 0,   'gap': 0,   'side': 'none'},
    (1, 'FRONT'):   {'desc': 'double_pulse_center','pulses': 2,  'ms': 100, 'gap': 200, 'side': 'both'},
    (1, 'LEFT'):    {'desc': 'double_pulse_left',  'pulses': 2,  'ms': 100, 'gap': 200, 'side': 'left'},
    (1, 'RIGHT'):   {'desc': 'double_pulse_right', 'pulses': 2,  'ms': 100, 'gap': 200, 'side': 'right'},
    (1, 'BEHIND'):  {'desc': 'single_pulse_both',  'pulses': 1,  'ms': 150, 'gap': 0,   'side': 'both'},
    (2, 'FRONT'):   {'desc': 'panic_continuous',   'pulses': 10, 'ms': 80,  'gap': 40,  'side': 'both'},
    (2, 'LEFT'):    {'desc': 'panic_left',         'pulses': 8,  'ms': 80,  'gap': 40,  'side': 'left'},
    (2, 'RIGHT'):   {'desc': 'panic_right',        'pulses': 8,  'ms': 80,  'gap': 40,  'side': 'right'},
    (2, 'BEHIND'):  {'desc': 'panic_both_strong',  'pulses': 10, 'ms': 80,  'gap': 40,  'side': 'both'},
}


class HapticFeedback(Node):
    """
    Haptic feedback simulation - translates alert levels to vibration commands.
    In a real deployment: replace _send_to_motor() with serial/GPIO calls.
    """

    def __init__(self) -> None:
        super().__init__('haptic_feedback')

        self.create_subscription(Int8,   '/smartnav/alert_level',    self._alert_cb, 10)
        self.create_subscription(String, '/smartnav/alert_direction', self._dir_cb,   10)

        self._pub = self.create_publisher(String, '/smartnav/haptic_pattern', 10)

        self._alert     = 0
        self._direction = 'NONE'
        self._last_key  = (-1, '')

        self.get_logger().info(
            'HapticFeedback node started | '
            'In simulation: publishing patterns to /smartnav/haptic_pattern'
        )

    # ────────────────────────────────────────────────────────────────
    def _alert_cb(self, msg: Int8) -> None:
        self._alert = msg.data
        self._process()

    def _dir_cb(self, msg: String) -> None:
        self._direction = msg.data

    # ────────────────────────────────────────────────────────────────
    def _process(self) -> None:
        """Translate current alert + direction to haptic command."""
        key = (self._alert, self._direction)
        if key == self._last_key:
            return  # no change

        self._last_key = key
        pattern = HAPTIC_PATTERNS.get(key, HAPTIC_PATTERNS.get((self._alert, 'FRONT')))

        if pattern is None or pattern['pulses'] == 0:
            self._send_to_motor('SILENCE', pattern or {})
            return

        self._send_to_motor(key, pattern)

    # ────────────────────────────────────────────────────────────────
    def _send_to_motor(self, key, pattern: dict) -> None:
        """
        Send haptic command.
        SIMULATION: log and publish.
        REAL DEPLOYMENT: replace with:
            serial.write(f"VIB:{pattern['side']}:{pattern['pulses']}:"
                         f"{pattern['ms']}:{pattern['gap']}\n".encode())
        """
        if not pattern or pattern.get('pulses', 0) == 0:
            msg = String()
            msg.data = 'SILENCE'
            self._pub.publish(msg)
            return

        cmd = (
            f"VIB | side={pattern['side']} "
            f"pulses={pattern['pulses']} "
            f"on_ms={pattern['ms']} "
            f"gap_ms={pattern['gap']} "
            f"pattern={pattern['desc']}"
        )
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)

        level_name = {0: 'SAFE', 1: 'WARNING', 2: 'DANGER'}.get(self._alert, '?')
        self.get_logger().info(
            f'[HAPTIC] {level_name}/{self._direction} -> {cmd}'
        )


# ──────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = HapticFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
