#!/usr/bin/env python3
"""
Relay ROS /cmd_vel to Gazebo VelocityControl via `gz topic`.

Why subprocess instead of ros_gz_bridge:
- gz-transport inter-process publisher discovery (multicast UDP) is unreliable
  on this machine (NodeShared RecvSrvRequest errors).  A fresh `gz topic -p`
  process per message always triggers a new discovery handshake that succeeds —
  the same mechanism used by reset_cars.sh to move the simulated cars.

Thread model:
- The ROS spin thread NEVER blocks.  It only stores the latest (vx, wz) and
  sets an Event to wake the worker.
- A single daemon worker thread handles all subprocess calls sequentially.
  This prevents process pile-up while keeping the ROS spin thread free.
"""

from __future__ import annotations

import subprocess
import threading

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

CMD_VEL_TOPIC = '/cmd_vel'
# Robot is now defined in city.world SDF with explicit absolute topic.
GZ_CMD_VEL_TOPIC = '/smartnav/cmd_vel'
# Higher rate so a STOP order is re-affirmed quickly if gz transport drops it.
HEARTBEAT_HZ = 5.0


class CmdVelRelay(Node):
    """Forward incoming ROS cmd_vel to Gazebo using a non-blocking worker thread."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_relay')

        self._lock = threading.Lock()
        self._vx: float = 0.0
        self._wz: float = 0.0
        self._last_sent_payload: str = ''
        # When True, the heartbeat forces a resend even if payload is unchanged,
        # ensuring that a velocity command (especially STOP) lost in gz transport
        # is delivered without waiting for the next state change.
        self._force_resend: bool = False

        # Event wakes the worker thread without blocking the spin thread.
        self._send_event = threading.Event()

        self._worker_thread = threading.Thread(
            target=self._worker, daemon=True
        )
        self._worker_thread.start()

        self._sub = self.create_subscription(
            Twist, CMD_VEL_TOPIC, self._on_cmd, 20
        )
        self._heartbeat = self.create_timer(1.0 / HEARTBEAT_HZ, self._on_heartbeat)

        self.get_logger().info(
            f'cmd_vel_relay actif: {CMD_VEL_TOPIC} -> {GZ_CMD_VEL_TOPIC} '
            '(daemon worker + gz topic subprocess).'
        )

    # ── Callbacks (run in ROS spin thread — must never block) ─────────────────

    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._vx = float(msg.linear.x)
            self._wz = float(msg.angular.z)
        self._send_event.set()

    def _on_heartbeat(self) -> None:
        """Periodically re-send current velocity (handles dropped messages)."""
        with self._lock:
            self._force_resend = True
        self._send_event.set()

    # ── Worker thread (blocks freely — separate from ROS spin) ────────────────

    def _worker(self) -> None:
        while True:
            self._send_event.wait()
            self._send_event.clear()

            with self._lock:
                vx = self._vx
                wz = self._wz
                force = self._force_resend
                self._force_resend = False

            payload = (
                f'linear: {{x: {vx:.4f} y: 0.0 z: 0.0}} '
                f'angular: {{x: 0.0 y: 0.0 z: {wz:.4f}}}'
            )

            # Skip only when payload is identical AND no heartbeat forced a resend.
            # This preserves the anti-spam dedup for the 20 Hz cmd stream while
            # ensuring the heartbeat can push a STOP that gz transport dropped.
            if payload == self._last_sent_payload and not force:
                continue

            self._last_sent_payload = payload
            self._gz_pub(payload)

    def _gz_pub(self, payload: str) -> None:
        """Publish one Twist message to Gazebo (blocking, runs in worker thread)."""
        try:
            proc = subprocess.Popen(
                [
                    'gz', 'topic',
                    '-t', GZ_CMD_VEL_TOPIC,
                    '-m', 'gz.msgs.Twist',
                    '-p', payload,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
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
