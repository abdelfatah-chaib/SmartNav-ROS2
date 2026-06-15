#!/usr/bin/env python3
"""
Reactive autonomous navigator for the SmartNav smart cane simulation.

FSM a 5 etats
──────────────
WANDER            : avance en ligne droite, surveille le LiDAR et le niveau d'alerte.
OBSTACLE_DETECTED : detecte un obstacle statique (<0.75 m), choisit le sens de rotation.
ROTATE_AND_SCAN   : tourne legerement pour mesurer l'espace disponible des deux cotes
                    avant de choisir la direction d'evitement.
AVOIDANCE         : tourne pour contourner l'obstacle statique (arbres, cylindres...).
DANGER_HOLD       : arret total d'urgence declenche par alert_level=2 (voiture, danger
                    imminent). Attend que la voie soit libre (alert_level=0) avant de
                    reprendre.

Transitions cles
────────────────
  WANDER -> DANGER_HOLD        : alert_level == 2
  WANDER -> OBSTACLE_DETECTED  : front_min < OBSTACLE_DISTANCE_M
  DANGER_HOLD -> WANDER        : alert_level == 0 confirme N cycles
  OBSTACLE_DETECTED -> ROTATE_AND_SCAN : immediat
  ROTATE_AND_SCAN -> AVOIDANCE : apres scan des deux cotes
  AVOIDANCE -> WANDER          : front_min > CLEAR_DISTANCE_M x CLEAR_CONFIRM_CYCLES
"""

from __future__ import annotations

from enum import auto, Enum
import math
import threading

from geometry_msgs.msg import Twist
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8, String


class NavState(Enum):
    WANDER = auto()
    OBSTACLE_DETECTED = auto()
    ROTATE_AND_SCAN = auto()
    AVOIDANCE = auto()
    DANGER_HOLD = auto()


MODE_AUTONOMOUS = 'autonomous'
SCAN_TOPIC = '/scan'
CMD_VEL_TOPIC = '/cmd_vel'
MODE_TOPIC = '/smartnav/mode'
# DANGER_HOLD (arret total) est reserve au danger imminent de VOITURE, publie
# par crossing_monitor_node sur /smartnav/crossing_alert. Les obstacles
# statiques (arbres, bancs...) detectes par le LiDAR passent uniquement par
# OBSTACLE_DETECTED -> ROTATE_AND_SCAN -> AVOIDANCE (contournement) et ne doivent jamais
# declencher DANGER_HOLD, sinon le robot reste bloque indefiniment devant un obstacle fixe.
ALERT_TOPIC = '/smartnav/crossing_alert'
# Topic de monitoring de l'etat FSM courant
STATE_TOPIC = '/smartnav/state'

CONTROL_HZ = 20.0
WANDER_LINEAR_X = 0.3
AVOIDANCE_ANGULAR_Z = 0.8
AVOIDANCE_ANGULAR_Z_MIN = 0.45
OBSTACLE_DISTANCE_M = 0.75
CLEAR_DISTANCE_M = 0.95
WATCHDOG_TIMEOUT_S = 0.5
WATCHDOG_STARTUP_GRACE_S = 20.0
MIN_AVOIDANCE_TIME_S = 1.5
CLEAR_CONFIRM_CYCLES = 10
MIN_VALID_SCAN_DISTANCE_M = 0.30
ROBUST_CLOSEST_SAMPLE_COUNT = 5

# Cycles consecutifs alert_level==0 requis avant de quitter DANGER_HOLD
DANGER_HOLD_CLEAR_CYCLES = 8

FRONT_CONE_DEG = 35.0
FRONT_HALF_PLANE_DEG = 90.0
SIDE_SECTOR_DEG = 50.0

# Duree de l'etat ROTATE_AND_SCAN (en secondes) pour mesurer les deux cotes
ROTATE_AND_SCAN_DURATION_S = 0.6


class SmartNavNavigator(Node):

    def __init__(self) -> None:
        super().__init__('smartnav_navigator')

        self._lock = threading.Lock()
        self._state = NavState.WANDER
        self._turn_direction = 1.0
        self._mode = MODE_AUTONOMOUS
        self._last_scan_stamp = None
        self._front_min = float('inf')
        self._front_half_min = float('inf')
        self._left_min = float('inf')
        self._right_min = float('inf')
        self._clear_cycles = 0
        self._danger_clear_cycles = 0
        self._alert_level = 0
        self._state_enter_time = self.get_clock().now()
        self._node_start_time = self.get_clock().now()
        # Mesures collectees pendant ROTATE_AND_SCAN
        self._scan_left_samples: list[float] = []
        self._scan_right_samples: list[float] = []

        self._scan_group = ReentrantCallbackGroup()
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._mode_group = ReentrantCallbackGroup()
        self._alert_group = ReentrantCallbackGroup()

        self._scan_sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self._on_scan, 10,
            callback_group=self._scan_group,
        )
        self._mode_sub = self.create_subscription(
            String, MODE_TOPIC, self._on_mode, 10,
            callback_group=self._mode_group,
        )
        self._alert_sub = self.create_subscription(
            Int8, ALERT_TOPIC, self._on_alert, 10,
            callback_group=self._alert_group,
        )
        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        # Publication de l'etat FSM courant pour monitoring en temps reel
        self._state_pub = self.create_publisher(String, STATE_TOPIC, 10)
        self._control_timer = self.create_timer(
            1.0 / CONTROL_HZ, self._control_loop,
            callback_group=self._control_group,
        )

        self.get_logger().info(
            'smartnav_navigator demarre — FSM 5 etats (ROTATE_AND_SCAN + DANGER_HOLD actifs).'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_mode(self, msg: String) -> None:
        with self._lock:
            self._mode = msg.data.strip().lower()
            if self._mode != MODE_AUTONOMOUS:
                self._transition_to(NavState.WANDER)
        if self._mode != MODE_AUTONOMOUS:
            self._publish_stop()

    def _on_scan(self, msg: LaserScan) -> None:
        front_min = self._sector_min(msg, 0.0, FRONT_CONE_DEG)
        front_half_min = self._sector_min(msg, 0.0, FRONT_HALF_PLANE_DEG)
        left_min = self._sector_min(msg, math.pi / 2.0, SIDE_SECTOR_DEG)
        right_min = self._sector_min(msg, -math.pi / 2.0, SIDE_SECTOR_DEG)
        with self._lock:
            self._front_min = front_min
            self._front_half_min = front_half_min
            self._left_min = left_min
            self._right_min = right_min
            self._last_scan_stamp = self.get_clock().now()
            # Collecte des echantillons pendant ROTATE_AND_SCAN
            if self._state == NavState.ROTATE_AND_SCAN:
                self._scan_left_samples.append(left_min)
                self._scan_right_samples.append(right_min)

    def _on_alert(self, msg: Int8) -> None:
        with self._lock:
            self._alert_level = int(msg.data)

    # ── Utilitaires ──────────────────────────────────────────────────────────

    def _sector_min(
        self, scan: LaserScan, center_rad: float, half_angle_deg: float
    ) -> float:
        sector_values = []
        half_angle_rad = math.radians(half_angle_deg)
        min_valid_distance = max(scan.range_min, MIN_VALID_SCAN_DISTANCE_M)
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value < min_valid_distance or value > scan.range_max:
                continue
            ray_angle = scan.angle_min + index * scan.angle_increment
            delta = math.atan2(
                math.sin(ray_angle - center_rad),
                math.cos(ray_angle - center_rad),
            )
            if abs(delta) <= half_angle_rad:
                sector_values.append(value)

        if not sector_values:
            return float('inf')
        sector_values.sort()
        sample_count = min(ROBUST_CLOSEST_SAMPLE_COUNT, len(sector_values))
        return sum(sector_values[:sample_count]) / sample_count

    def _watchdog_expired(self) -> bool:
        node_age_s = (self.get_clock().now() - self._node_start_time).nanoseconds / 1e9
        if node_age_s < WATCHDOG_STARTUP_GRACE_S:
            return False
        with self._lock:
            if self._last_scan_stamp is None:
                return True
            age_s = (self.get_clock().now() - self._last_scan_stamp).nanoseconds / 1e9
        return age_s > WATCHDOG_TIMEOUT_S

    def _transition_to(self, state: NavState) -> None:
        if state != self._state:
            self.get_logger().info(f'[FSM] {self._state.name} -> {state.name}')
        self._state = state
        self._state_enter_time = self.get_clock().now()
        if state not in (NavState.AVOIDANCE, NavState.DANGER_HOLD):
            self._clear_cycles = 0
        if state != NavState.DANGER_HOLD:
            self._danger_clear_cycles = 0
        if state == NavState.ROTATE_AND_SCAN:
            self._scan_left_samples = []
            self._scan_right_samples = []
        # Publie l'etat FSM sur le topic de monitoring
        state_msg = String()
        state_msg.data = state.name
        self._state_pub.publish(state_msg)

    def _state_elapsed_s(self) -> float:
        return (self.get_clock().now() - self._state_enter_time).nanoseconds / 1e9

    # ── Boucle de controle ───────────────────────────────────────────────────

    def _control_loop(self) -> None:
        if self._watchdog_expired():
            self.get_logger().warn('Watchdog LiDAR : pas de scan > 0.5 s, arret securite.')
            self._publish_stop()
            return

        with self._lock:
            mode = self._mode
            front_min = self._front_min
            front_half_min = self._front_half_min
            left_min = self._left_min
            right_min = self._right_min
            state = self._state
            alert_level = self._alert_level
            clear_cycles = self._clear_cycles
            danger_clear = self._danger_clear_cycles

        if mode != MODE_AUTONOMOUS:
            self._publish_stop()
            return

        # ── WANDER ──────────────────────────────────────────────────────────
        if state == NavState.WANDER:
            if alert_level == 2:
                with self._lock:
                    self._transition_to(NavState.DANGER_HOLD)
                self._publish_stop()
                self.get_logger().error(
                    '[DANGER_HOLD] Danger imminent detecte — arret total.'
                )
                return

            if front_min < OBSTACLE_DISTANCE_M:
                with self._lock:
                    self._transition_to(NavState.OBSTACLE_DETECTED)
                self._publish_stop()
                return

            self._publish_cmd(WANDER_LINEAR_X, 0.0)
            return

        # ── DANGER_HOLD ──────────────────────────────────────────────────────
        if state == NavState.DANGER_HOLD:
            self._publish_stop()

            if alert_level == 0:
                danger_clear += 1
            else:
                danger_clear = 0

            with self._lock:
                self._danger_clear_cycles = danger_clear

            if danger_clear >= DANGER_HOLD_CLEAR_CYCLES:
                with self._lock:
                    self._transition_to(NavState.WANDER)
                self.get_logger().info(
                    '[DANGER_HOLD] Voie libre confirmee — reprise de la marche.'
                )
            return

        # ── OBSTACLE_DETECTED ────────────────────────────────────────────────
        if state == NavState.OBSTACLE_DETECTED:
            # Transition vers ROTATE_AND_SCAN pour mesurer les deux cotes
            with self._lock:
                self._transition_to(NavState.ROTATE_AND_SCAN)
            self._publish_stop()
            return

        # ── ROTATE_AND_SCAN ──────────────────────────────────────────────────
        if state == NavState.ROTATE_AND_SCAN:
            # Tourne legerement pendant ROTATE_AND_SCAN_DURATION_S pour scanner
            elapsed = self._state_elapsed_s()
            if elapsed < ROTATE_AND_SCAN_DURATION_S:
                # Tourne doucement a gauche pour scanner les deux cotes
                self._publish_cmd(0.0, 0.3)
                return

            # Analyse des echantillons collectes pour choisir le meilleur cote
            with self._lock:
                left_samples = list(self._scan_left_samples)
                right_samples = list(self._scan_right_samples)

            avg_left = sum(left_samples) / len(left_samples) if left_samples else left_min
            avg_right = sum(right_samples) / len(right_samples) if right_samples else right_min

            self.get_logger().info(
                f'[ROTATE_AND_SCAN] Espace moyen : gauche={avg_left:.2f}m '
                f'droite={avg_right:.2f}m — choix: {"GAUCHE" if avg_left >= avg_right else "DROITE"}'
            )

            with self._lock:
                self._turn_direction = 1.0 if avg_left >= avg_right else -1.0
                self._transition_to(NavState.AVOIDANCE)
            self._publish_stop()
            return

        # ── AVOIDANCE ────────────────────────────────────────────────────────
        if state == NavState.AVOIDANCE:
            if alert_level == 2:
                with self._lock:
                    self._transition_to(NavState.DANGER_HOLD)
                self._publish_stop()
                return

            # Exit condition: the WIDE cone (±90°) must be clear, not just the
            # narrow front cone (±35°).  Without this, the robot turns ~36° and
            # front_min jumps to inf (tree exits the narrow cone) → FSM exits
            # AVOIDANCE prematurely and the robot immediately drives back into
            # the tree.
            if front_half_min >= CLEAR_DISTANCE_M:
                clear_cycles += 1
            else:
                clear_cycles = 0

            with self._lock:
                self._clear_cycles = clear_cycles
                turn = self._turn_direction

            min_time_ok = self._state_elapsed_s() >= MIN_AVOIDANCE_TIME_S
            if clear_cycles >= CLEAR_CONFIRM_CYCLES and min_time_ok:
                with self._lock:
                    self._transition_to(NavState.WANDER)
                self._publish_cmd(WANDER_LINEAR_X, 0.0)
                return

            clearance_ratio = min(max(front_min / CLEAR_DISTANCE_M, 0.0), 1.0)
            angular_speed = (
                AVOIDANCE_ANGULAR_Z
                - (AVOIDANCE_ANGULAR_Z - AVOIDANCE_ANGULAR_Z_MIN) * clearance_ratio
            )
            self._publish_cmd(0.0, turn * angular_speed)
            return

    # ── Publication ──────────────────────────────────────────────────────────

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmartNavNavigator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node._publish_stop()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
