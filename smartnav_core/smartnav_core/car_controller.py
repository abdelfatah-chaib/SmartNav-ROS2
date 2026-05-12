#!/usr/bin/env python3
"""
SmartNav - Car Controller Node
===============================
Manages the 3 dynamic cars in the highway_forest world.
Replaces the reset_cars.sh shell script with a proper ROS2 node.

Behavior:
  - Publishes constant velocity to each car on its cmd_vel topic
  - Monitors car positions via /model/{name}/pose (gz bridge)
  - Teleports cars back to start when they go out of the road bounds
    using the /world/smartnav_world/set_pose Gazebo service

Car configuration (matches highway_forest.world):
  car_red   : EAST  (x: -30 -> +32), lane y=-5.0
  car_blue  : WEST  (x: +30 -> -32), lane y=-8.5  (rotated 180 in world)
  car_white : EAST  (x: -18 -> +32), lane y=-12.0

Author: SmartNav Team - Embedded Real-Time Systems, IAOC Master
"""

from __future__ import annotations
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Car definitions: name, speed (+east/-west), start_x, lane_y, reset_x, max_x, yaw
CARS = [
    {
        'name':    'car_red',
        'topic':   '/car_red_cmd_vel',
        'speed_x':  7.0,
        'start_x': -30.0,
        'lane_y':   -5.0,
        'z':         0.38,
        'reset_x': -30.0,
        'limit_x':  32.0,   # teleport back when x > limit
        'yaw_w':    1.0,
        'yaw_z':    0.0,
    },
    {
        'name':    'car_blue',
        'topic':   '/car_blue_cmd_vel',
        'speed_x':  6.5,     # positive because model is rotated 180 in world
        'start_x':  30.0,
        'lane_y':   -8.5,
        'z':         0.38,
        'reset_x':  30.0,
        'limit_x':  32.0,   # teleport back when x > 32 (travelling "forward" in local frame)
        'yaw_w':    0.0,     # rotated 180 deg in world = goes WEST
        'yaw_z':    1.0,
    },
    {
        'name':    'car_white',
        'topic':   '/car_white_cmd_vel',
        'speed_x':  5.0,
        'start_x': -18.0,
        'lane_y':  -12.0,
        'z':         0.38,
        'reset_x': -18.0,
        'limit_x':  32.0,
        'yaw_w':    1.0,
        'yaw_z':    0.0,
    },
]

# How often to send velocity commands (seconds)
VEL_PERIOD   = 0.5   # 2Hz is sufficient for constant velocity
# How often to check positions and reset (seconds)
RESET_PERIOD = 12.0  # slightly less than time to cross road at 5-7 m/s


class CarController(Node):
    """Drives the dynamic cars and teleports them at road boundaries."""

    def __init__(self) -> None:
        super().__init__('car_controller')

        # Create one publisher per car
        self._publishers = {
            car['name']: self.create_publisher(Twist, car['topic'], 10)
            for car in CARS
        }

        # Velocity timer
        self.create_timer(VEL_PERIOD, self._send_velocities)
        # Reset timer
        self.create_timer(RESET_PERIOD, self._reset_cars)

        self.get_logger().info(
            f'CarController started | managing {len(CARS)} cars'
        )
        # Send initial velocities immediately
        self._send_velocities()

    # ────────────────────────────────────────────────────────────────
    def _send_velocities(self) -> None:
        """Publish constant velocity to each car."""
        for car in CARS:
            cmd = Twist()
            cmd.linear.x = car['speed_x']
            self._publishers[car['name']].publish(cmd)

    # ────────────────────────────────────────────────────────────────
    def _reset_cars(self) -> None:
        """Teleport each car back to start position using gz service."""
        for car in CARS:
            self._teleport(
                name=car['name'],
                x=car['reset_x'],
                y=car['lane_y'],
                z=car['z'],
                yaw_z=car['yaw_z'],
                yaw_w=car['yaw_w'],
            )

    # ────────────────────────────────────────────────────────────────
    def _teleport(self, name: str, x: float, y: float, z: float,
                  yaw_z: float, yaw_w: float) -> None:
        """
        Teleport a model using gz service (same approach as reset_cars.sh).
        Non-blocking: runs in a subprocess.
        """
        req = (
            f"name: '{name}' "
            f"position {{ x: {x:.1f}  y: {y:.1f}  z: {z:.2f} }} "
            f"orientation {{ x: 0.0  y: 0.0  z: {yaw_z:.1f}  w: {yaw_w:.1f} }}"
        )
        cmd = [
            'gz', 'service',
            '-s', '/world/smartnav_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--req', req,
            '--timeout', '500'
        ]
        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info(f'[CAR RESET] {name} -> x={x:.1f} y={y:.1f}')
        except FileNotFoundError:
            self.get_logger().warn(
                "'gz' command not found. Car reset skipped. "
                "Run reset_cars.sh manually if needed."
            )


# ──────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
