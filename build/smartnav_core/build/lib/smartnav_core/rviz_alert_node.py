#!/usr/bin/env python3
"""
Canne SmartNav — Visualisation RViz.

Abonne :
  /smartnav/alert_level  (Int8)    -> couleur du marqueur canne
  /smartnav/vibration    (Int8)    -> oscillation visuelle de la canne
  /smartnav/battery      (Float32) -> niveau batterie (marqueur barre)

Publie :
  /smartnav/canne_marker  (Marker) -- cylindre canne colore + effet vibration
  /smartnav/alert_text    (Marker) -- texte flottant au-dessus de la canne
  /smartnav/battery_marker(Marker) -- barre de progression batterie
  /smartnav/trail_marker  (Marker) -- trajectoire passee du robot (LINE_STRIP)
  /smartnav/zone_markers  (MarkerArray) -- zones d'alerte concentriques

Ameliorations v2:
- Marqueur batterie : barre de progression visuelle au-dessus du robot
- Trajectoire passee : LINE_STRIP pour visualiser le chemin parcouru
- Zones d'alerte : cercles concentriques (vert 1.2m, orange 0.5m)
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float32, Int8
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# Amplitude de l'oscillation visuelle selon le niveau de vibration (m)
_VIBRATION_AMPLITUDE = {0: 0.0, 1: 0.012, 2: 0.025}
# Frequence d'oscillation (Hz)
_VIBRATION_FREQ = {0: 0.0, 1: 4.0, 2: 10.0}

_ALERT_TEXT = {
    0: 'Voie libre',
    1: 'ATTENTION — Obstacle !',
    2: 'DANGER — ARRET REQUIS !',
}

# Longueur maximale de la trajectoire memorisee (en points)
MAX_TRAIL_POINTS = 200


class RvizAlertNode(Node):

    def __init__(self):
        super().__init__('rviz_alert_node')

        self.alert_level = 0
        self.vibration_level = 0
        self._prev_alert_level = -1
        self.blink_state = True
        self._phase = 0.0
        self._battery_level = 100.0
        # Historique de positions pour la trajectoire
        self._trail_points: list[Point] = []

        self.subscription = self.create_subscription(
            Int8,
            '/smartnav/alert_level',
            self.alert_callback,
            10,
        )
        self.vibration_sub = self.create_subscription(
            Int8,
            '/smartnav/vibration',
            self._vibration_callback,
            10,
        )
        self._battery_sub = self.create_subscription(
            Float32,
            '/smartnav/battery',
            self._battery_callback,
            10,
        )

        self.marker_pub = self.create_publisher(Marker, '/smartnav/canne_marker', 10)
        self.text_pub = self.create_publisher(Marker, '/smartnav/alert_text', 10)
        self._battery_pub = self.create_publisher(Marker, '/smartnav/battery_marker', 10)
        self._trail_pub = self.create_publisher(Marker, '/smartnav/trail_marker', 10)
        self._zone_pub = self.create_publisher(MarkerArray, '/smartnav/zone_markers', 10)

        self.timer = self.create_timer(0.1, self.publish_markers)

        self.get_logger().info('Noeud visualisation RViz demarre (v2 — batterie + trajectoire).')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def alert_callback(self, msg: Int8) -> None:
        self.alert_level = msg.data
        if self.alert_level == self._prev_alert_level:
            return
        self._prev_alert_level = self.alert_level
        if self.alert_level == 0:
            self.get_logger().info('Voie libre')
        elif self.alert_level == 1:
            self.get_logger().warn('Attention — obstacle eloigne')
        elif self.alert_level == 2:
            self.get_logger().error('DANGER IMMINENT — Arret requis !')

    def _vibration_callback(self, msg: Int8) -> None:
        self.vibration_level = int(msg.data)

    def _battery_callback(self, msg: Float32) -> None:
        self._battery_level = float(msg.data)

    # ── Publication des marqueurs ─────────────────────────────────────────────

    def publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()

        freq = _VIBRATION_FREQ.get(self.vibration_level, 0.0)
        self._phase = (self._phase + 2.0 * math.pi * freq * 0.1) % (2.0 * math.pi)

        amplitude = _VIBRATION_AMPLITUDE.get(self.vibration_level, 0.0)
        offset_y = amplitude * math.sin(self._phase)

        # ── Marqueur canne (cylindre) ───────────────────────────────────────
        canne = Marker()
        canne.header.frame_id = 'base_link'
        canne.header.stamp = now
        canne.ns = 'canne'
        canne.id = 0
        canne.type = Marker.CYLINDER
        canne.action = Marker.ADD
        canne.scale.x = 0.05
        canne.scale.y = 0.05
        canne.scale.z = 1.0
        canne.pose.position.x = 0.0
        canne.pose.position.y = offset_y
        canne.pose.position.z = 0.5
        canne.pose.orientation.w = 1.0

        if self.alert_level == 0:
            canne.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        elif self.alert_level == 1:
            canne.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        elif self.alert_level == 2:
            self.blink_state = not self.blink_state
            alpha = 1.0 if self.blink_state else 0.2
            canne.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=alpha)

        self.marker_pub.publish(canne)

        # ── Marqueur texte flottant ─────────────────────────────────────────
        text = Marker()
        text.header.frame_id = 'base_link'
        text.header.stamp = now
        text.ns = 'canne_text'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.20
        text.pose.position.x = 0.0
        text.pose.position.y = offset_y
        text.pose.position.z = 1.25
        text.pose.orientation.w = 1.0
        text.text = _ALERT_TEXT.get(self.alert_level, '')

        if self.alert_level == 0:
            text.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.9)
        elif self.alert_level == 1:
            text.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
        elif self.alert_level == 2:
            alpha_t = 1.0 if self.blink_state else 0.2
            text.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=alpha_t)

        self.text_pub.publish(text)

        # ── Marqueur batterie (barre de progression) ────────────────────────
        self._publish_battery_marker(now)

        # ── Zones d'alerte concentriques ────────────────────────────────────
        self._publish_zone_markers(now)

        # ── Trajectoire passee (LINE_STRIP) ─────────────────────────────────
        self._publish_trail_marker(now)

    def _publish_battery_marker(self, now) -> None:
        """Barre de progression batterie au-dessus du robot."""
        batt = self._battery_level / 100.0  # 0.0 -> 1.0

        # Fond de la barre (gris)
        bg = Marker()
        bg.header.frame_id = 'base_link'
        bg.header.stamp = now
        bg.ns = 'battery'
        bg.id = 10
        bg.type = Marker.CUBE
        bg.action = Marker.ADD
        bg.scale.x = 0.3
        bg.scale.y = 0.05
        bg.scale.z = 0.03
        bg.pose.position.x = 0.0
        bg.pose.position.y = 0.0
        bg.pose.position.z = 1.6
        bg.pose.orientation.w = 1.0
        bg.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.7)
        self._battery_pub.publish(bg)

        # Niveau de remplissage
        fill = Marker()
        fill.header.frame_id = 'base_link'
        fill.header.stamp = now
        fill.ns = 'battery'
        fill.id = 11
        fill.type = Marker.CUBE
        fill.action = Marker.ADD
        fill.scale.x = max(0.01, 0.3 * batt)
        fill.scale.y = 0.04
        fill.scale.z = 0.025
        fill.pose.position.x = -0.15 + fill.scale.x / 2.0
        fill.pose.position.y = 0.0
        fill.pose.position.z = 1.6
        fill.pose.orientation.w = 1.0

        if batt > 0.5:
            fill.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
        elif batt > 0.2:
            fill.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9)
        else:
            fill.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        self._battery_pub.publish(fill)

        # Texte du pourcentage
        batt_text = Marker()
        batt_text.header.frame_id = 'base_link'
        batt_text.header.stamp = now
        batt_text.ns = 'battery'
        batt_text.id = 12
        batt_text.type = Marker.TEXT_VIEW_FACING
        batt_text.action = Marker.ADD
        batt_text.scale.z = 0.10
        batt_text.pose.position.x = 0.0
        batt_text.pose.position.y = 0.0
        batt_text.pose.position.z = 1.72
        batt_text.pose.orientation.w = 1.0
        batt_text.text = f'{self._battery_level:.0f}%'
        batt_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        self._battery_pub.publish(batt_text)

    def _publish_zone_markers(self, now) -> None:
        """Zones d'alerte concentriques autour du robot."""
        markers = MarkerArray()

        zones = [
            (1.2, ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.15), 'zone_warn', 20),
            (0.5, ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.20), 'zone_danger', 21),
        ]

        for radius, color, ns, marker_id in zones:
            zone = Marker()
            zone.header.frame_id = 'base_link'
            zone.header.stamp = now
            zone.ns = ns
            zone.id = marker_id
            zone.type = Marker.CYLINDER
            zone.action = Marker.ADD
            zone.scale.x = radius * 2.0
            zone.scale.y = radius * 2.0
            zone.scale.z = 0.02
            zone.pose.position.x = 0.0
            zone.pose.position.y = 0.0
            zone.pose.position.z = 0.01
            zone.pose.orientation.w = 1.0
            zone.color = color
            markers.markers.append(zone)

        self._zone_pub.publish(markers)

    def _publish_trail_marker(self, now) -> None:
        """Trajectoire passee du robot sous forme de LINE_STRIP."""
        # Ajoute la position actuelle (origine dans base_link)
        p = Point()
        p.x = 0.0
        p.y = 0.0
        p.z = 0.05
        self._trail_points.append(p)

        # Limite la longueur de la trajectoire
        if len(self._trail_points) > MAX_TRAIL_POINTS:
            self._trail_points = self._trail_points[-MAX_TRAIL_POINTS:]

        trail = Marker()
        trail.header.frame_id = 'odom'
        trail.header.stamp = now
        trail.ns = 'trail'
        trail.id = 30
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.scale.x = 0.03
        trail.pose.orientation.w = 1.0
        trail.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.7)
        trail.points = self._trail_points
        self._trail_pub.publish(trail)


def main(args=None):
    rclpy.init(args=args)
    node = RvizAlertNode()
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
