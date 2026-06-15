#!/usr/bin/env python3
"""
SmartNav — Noeud simulateur de batterie.

Simule un niveau de batterie qui decroit avec le temps de simulation.
Publie sur /smartnav/battery (Float32, 0.0–100.0%).
En dessous de 20%, publie un warning dans les logs.
Purement visuel/demonstratif — ne modifie pas la logique de navigation.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Parametres par defaut
DEFAULT_INITIAL_LEVEL = 100.0   # % de depart
DEFAULT_DRAIN_RATE = 0.5        # % par minute (ajustable)
DEFAULT_PUBLISH_HZ = 1.0        # frequence de publication
LOW_BATTERY_THRESHOLD = 20.0    # % seuil d'avertissement
CRITICAL_BATTERY_THRESHOLD = 5.0


class BatterySimulatorNode(Node):
    """Simule la decharge d'une batterie et publie le niveau en continu."""

    def __init__(self) -> None:
        super().__init__('battery_simulator_node')

        self.declare_parameter('initial_level', DEFAULT_INITIAL_LEVEL)
        self.declare_parameter('drain_rate_per_min', DEFAULT_DRAIN_RATE)
        self.declare_parameter('publish_hz', DEFAULT_PUBLISH_HZ)

        self._level = float(self.get_parameter('initial_level').value)
        self._low_battery_warned = False
        self._critical_battery_warned = False

        self._battery_pub = self.create_publisher(Float32, '/smartnav/battery', 10)

        publish_hz = float(self.get_parameter('publish_hz').value)
        self._timer = self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info(
            f'battery_simulator_node demarre — niveau initial: {self._level:.1f}% '
            f'drain: {DEFAULT_DRAIN_RATE:.1f}%/min'
        )

    def _tick(self) -> None:
        drain_rate = float(self.get_parameter('drain_rate_per_min').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        # Decharge proportionnelle au temps ecoule depuis le dernier tick
        drain_per_tick = drain_rate / 60.0 / publish_hz
        self._level = max(0.0, self._level - drain_per_tick)

        # Alertes de niveau bas
        if self._level <= CRITICAL_BATTERY_THRESHOLD and not self._critical_battery_warned:
            self.get_logger().error(
                f'[BATTERIE] CRITIQUE : {self._level:.1f}% — recharge immediate requise !'
            )
            self._critical_battery_warned = True
        elif self._level <= LOW_BATTERY_THRESHOLD and not self._low_battery_warned:
            self.get_logger().warn(
                f'[BATTERIE] Faible : {self._level:.1f}% — recharge recommandee.'
            )
            self._low_battery_warned = True

        # Publication
        msg = Float32()
        msg.data = self._level
        self._battery_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatterySimulatorNode()
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
