import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8

MIN_VALID_SCAN_DISTANCE_M = 0.30
ROBUST_CLOSEST_SAMPLE_COUNT = 5
# Valeur par defaut des parametres ROS (configurables via ROS params)
DEFAULT_FRONT_CONE_DEG = 35.0
DEFAULT_DANGER_THRESHOLD_M = 0.5
DEFAULT_WARN_THRESHOLD_M = 1.2
# Taille du filtre median temporel (nombre de scans consecutifs)
MEDIAN_FILTER_SIZE = 3


class ObstacleDetector(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_detector')

        # Parametres configurables via ROS (sans recompilation)
        self.declare_parameter('front_cone_deg', DEFAULT_FRONT_CONE_DEG)
        self.declare_parameter('danger_threshold_m', DEFAULT_DANGER_THRESHOLD_M)
        self.declare_parameter('warn_threshold_m', DEFAULT_WARN_THRESHOLD_M)

        self.alert_publisher = self.create_publisher(Int8, '/smartnav/alert_level', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )
        self._prev_alert = -1
        # Filtre median temporel : memorise les N derniers niveaux d'alerte
        self._alert_history: deque[int] = deque(maxlen=MEDIAN_FILTER_SIZE)

    def scan_callback(self, msg: LaserScan) -> None:
        # Lecture des parametres dynamiques
        front_cone_deg = float(self.get_parameter('front_cone_deg').value)
        danger_threshold_m = float(self.get_parameter('danger_threshold_m').value)
        warn_threshold_m = float(self.get_parameter('warn_threshold_m').value)

        half_cone_rad = math.radians(front_cone_deg)
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

        # Calcul du niveau d'alerte brut
        if min_distance <= danger_threshold_m:
            raw_level = 2
        elif min_distance <= warn_threshold_m:
            raw_level = 1
        else:
            raw_level = 0

        # Filtre median temporel : evite les fausses alertes dues a des rayons parasites
        self._alert_history.append(raw_level)
        sorted_history = sorted(self._alert_history)
        median_level = sorted_history[len(sorted_history) // 2]

        alert_msg = Int8()
        alert_msg.data = median_level

        if median_level != self._prev_alert:
            if median_level == 2:
                self.get_logger().error(
                    f'[DANGER] Danger imminent a {min_distance:.2f}m ! Arret requis.'
                )
            elif median_level == 1:
                self.get_logger().warn(
                    f'[ATTENTION] Obstacle detecte a {min_distance:.2f}m.'
                )
            else:
                self.get_logger().info(
                    f'[INFO] Voie libre. Obstacle le plus proche a {min_distance:.2f}m.'
                )
            self._prev_alert = median_level

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
