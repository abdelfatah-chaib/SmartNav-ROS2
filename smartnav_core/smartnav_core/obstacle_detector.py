import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8


class ObstacleDetector(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_detector')
        self.alert_publisher = self.create_publisher(Int8, '/smartnav/alert_level', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

    def scan_callback(self, msg: LaserScan) -> None:
        valid_ranges = [
            distance
            for distance in msg.ranges
            if math.isfinite(distance) and distance >= 0.1
        ]

        if valid_ranges:
            min_distance = min(valid_ranges)
        else:
            min_distance = float('inf')

        alert_msg = Int8()

        if min_distance <= 0.5:
            alert_msg.data = 2
            self.get_logger().error(
                f'[DANGER] Danger imminent au sol ou en hauteur a {min_distance:.2f}m ! Arret requis.'
            )
        elif min_distance <= 1.2:
            alert_msg.data = 1
            self.get_logger().warn(
                f'[ATTENTION] Obstacle detecte a {min_distance:.2f}m.'
            )
        else:
            alert_msg.data = 0
            self.get_logger().info(
                f'[INFO] Voie libre. Obstacle le plus proche a {min_distance:.2f}m.'
            )

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
