#!/usr/bin/env python3
"""
Canne SmartNav — Visualisation RViz.

Abonne :
  /smartnav/alert_level  (Int8)  -> couleur du marqueur canne
  /smartnav/vibration    (Int8)  -> oscillation visuelle de la canne

Publie :
  /smartnav/canne_marker  (Marker) -- cylindre canne colore + effet vibration
  /smartnav/alert_text    (Marker) -- texte flottant au-dessus de la canne
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Int8
from visualization_msgs.msg import Marker


# Amplitude de l'oscillation visuelle selon le niveau de vibration (m)
_VIBRATION_AMPLITUDE = {0: 0.0, 1: 0.012, 2: 0.025}
# Frequence d'oscillation (Hz)
_VIBRATION_FREQ = {0: 0.0, 1: 4.0, 2: 10.0}

_ALERT_TEXT = {
    0: 'Voie libre',
    1: 'ATTENTION — Obstacle !',
    2: 'DANGER — ARRET REQUIS !',
}


class RvizAlertNode(Node):

    def __init__(self):
        super().__init__('rviz_alert_node')

        self.alert_level = 0
        self.vibration_level = 0
        self._prev_alert_level = -1
        self.blink_state = True
        self._phase = 0.0

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

        self.marker_pub = self.create_publisher(Marker, '/smartnav/canne_marker', 10)
        self.text_pub = self.create_publisher(Marker, '/smartnav/alert_text', 10)

        self.timer = self.create_timer(0.1, self.publish_markers)

        self.get_logger().info('Noeud visualisation RViz demarre.')

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
