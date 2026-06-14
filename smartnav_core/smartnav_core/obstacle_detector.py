#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8

ROBUST_CLOSEST_SAMPLE_COUNT = 5


class ObstacleDetector(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_detector')

        # ── Paramètres dynamiques (modifiables sans recompiler) ──────────────
        self.declare_parameter('front_cone_deg', 35.0)
        self.declare_parameter('min_valid_distance_m', 0.30)
        self.declare_parameter('danger_distance_m', 0.5)
        self.declare_parameter('warn_distance_m', 1.2)

        self.alert_publisher = self.create_publisher(Int8, '/smartnav/alert_level', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )
        self._prev_alert = -1
        self.get_logger().info('obstacle_detector démarré avec paramètres dynamiques.')

    def scan_callback(self, msg: LaserScan) -> None:
        # Lire les paramètres à chaque cycle (support ros2 param set en live)
        front_cone_deg = float(self.get_parameter('front_cone_deg').value)
        min_valid_distance = max(
            msg.range_min,
            float(self.get_parameter('min_valid_distance_m').value)
        )
        danger_dist = float(self.get_parameter('danger_distance_m').value)
        warn_dist = float(self.get_parameter('warn_distance_m').value)

        half_cone_rad = math.radians(front_cone_deg)
        valid_ranges = [
            distance for index, distance in enumerate(msg.ranges)
            if (
                math.isfinite(distance)
                and min_valid_distance <= distance <= msg.range_max
                and abs(msg.angle_min + index * msg.angle_increment) <= half_cone_rad
            )
        ]

        if valid_ranges:
            valid_ranges.sort()
            sample_count = min(ROBUST_CLOSEST_SAMPLE_COUNT, len(valid_ranges))
            min_distance = sum(valid_ranges[:sample_count]) / sample_count
        else:
            min_distance = float('inf')

        alert_msg = Int8()
        if min_distance <= danger_dist:
            alert_msg.data = 2
        elif min_distance <= warn_dist:
            alert_msg.data = 1
        else:
            alert_msg.data = 0

        if alert_msg.data != self._prev_alert:
            if alert_msg.data == 2:
                self.get_logger().error(
                    f'[DANGER] Obstacle à {min_distance:.2f}m ! Arrêt requis.'
                )
            elif alert_msg.data == 1:
                self.get_logger().warn(
                    f'[ATTENTION] Obstacle détecté à {min_distance:.2f}m.'
                )
            else:
                self.get_logger().info(
                    f'[INFO] Voie libre. Distance min : {min_distance:.2f}m.'
                )
            self._prev_alert = alert_msg.data

        self.alert_publisher.publish(alert_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleDetector()
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
