import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8

MIN_VALID_SCAN_DISTANCE_M = 0.30
ROBUST_CLOSEST_SAMPLE_COUNT = 5
FRONT_CONE_DEG = 35.0


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
        self._prev_alert = -1

    def scan_callback(self, msg: LaserScan) -> None:
        half_cone_rad = math.radians(FRONT_CONE_DEG)
        min_valid_distance = max(msg.range_min, MIN_VALID_SCAN_DISTANCE_M)
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

        if min_distance <= 0.5:
            alert_msg.data = 2
        elif min_distance <= 1.2:
            alert_msg.data = 1
        else:
            alert_msg.data = 0

        if alert_msg.data != self._prev_alert:
            if alert_msg.data == 2:
                self.get_logger().error(
                    f'[DANGER] Danger imminent a {min_distance:.2f}m ! Arret requis.'
                )
            elif alert_msg.data == 1:
                self.get_logger().warn(
                    f'[ATTENTION] Obstacle detecte a {min_distance:.2f}m.'
                )
            else:
                self.get_logger().info(
                    f'[INFO] Voie libre. Obstacle le plus proche a {min_distance:.2f}m.'
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
