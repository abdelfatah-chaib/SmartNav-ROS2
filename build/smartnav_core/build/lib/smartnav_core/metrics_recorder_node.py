#!/usr/bin/env python3
"""
SmartNav — Noeud d'enregistrement des metriques de session.

Abonne :
  /smartnav/alert_level    (Int8)   -> niveau d'alerte LiDAR
  /smartnav/crossing_alert (Int8)   -> alerte voiture
  /smartnav/state          (String) -> etat FSM courant

Enregistre dans un fichier CSV horodate :
  - Timestamp de chaque changement d'alerte
  - Duree passee dans chaque etat FSM
  - Nombre de detections danger/attention par session

Le fichier CSV est ecrit dans /tmp/smartnav_metrics_<date>.csv
et peut etre lu apres la simulation pour produire des graphiques.
"""

from __future__ import annotations

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String

DEFAULT_OUTPUT_DIR = '/tmp'


class MetricsRecorderNode(Node):
    """Enregistre les metriques de session SmartNav dans un fichier CSV."""

    def __init__(self) -> None:
        super().__init__('metrics_recorder_node')

        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)

        # Compteurs de session
        self._danger_count = 0
        self._attention_count = 0
        self._prev_alert_level = -1
        self._prev_crossing_level = -1
        self._prev_state = ''
        self._state_enter_time: datetime | None = None

        # Ouverture du fichier CSV
        output_dir = str(self.get_parameter('output_dir').value)
        session_ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(output_dir, f'smartnav_metrics_{session_ts}.csv')
        self._csv_file = open(self._csv_path, 'w', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp', 'event_type', 'value', 'detail',
            'session_danger_count', 'session_attention_count'
        ])
        self._csv_file.flush()

        # Abonnements
        self.create_subscription(Int8, '/smartnav/alert_level', self._on_alert, 10)
        self.create_subscription(Int8, '/smartnav/crossing_alert', self._on_crossing, 10)
        self.create_subscription(String, '/smartnav/state', self._on_state, 10)

        # Timer de summary toutes les 30 s
        self._summary_timer = self.create_timer(30.0, self._log_summary)

        self.get_logger().info(
            f'metrics_recorder_node demarre — CSV: {self._csv_path}'
        )

    def _write_row(self, event_type: str, value: str, detail: str = '') -> None:
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self._csv_writer.writerow([
            ts, event_type, value, detail,
            self._danger_count, self._attention_count
        ])
        self._csv_file.flush()

    def _on_alert(self, msg: Int8) -> None:
        level = int(msg.data)
        if level == self._prev_alert_level:
            return
        self._prev_alert_level = level

        if level == 2:
            self._danger_count += 1
            self._write_row('LIDAR_ALERT', 'DANGER', f'total_danger={self._danger_count}')
        elif level == 1:
            self._attention_count += 1
            self._write_row('LIDAR_ALERT', 'ATTENTION',
                            f'total_attention={self._attention_count}')
        else:
            self._write_row('LIDAR_ALERT', 'LIBRE')

    def _on_crossing(self, msg: Int8) -> None:
        level = int(msg.data)
        if level == self._prev_crossing_level:
            return
        self._prev_crossing_level = level

        level_str = {2: 'DANGER', 1: 'ATTENTION', 0: 'LIBRE'}.get(level, str(level))
        if level == 2:
            self._danger_count += 1
        self._write_row('CROSSING_ALERT', level_str)

    def _on_state(self, msg: String) -> None:
        new_state = msg.data
        if new_state == self._prev_state:
            return

        now = datetime.now()
        # Calcule la duree passee dans l'etat precedent
        duration_s = ''
        if self._state_enter_time is not None and self._prev_state:
            delta = (now - self._state_enter_time).total_seconds()
            duration_s = f'{delta:.2f}s'
            self._write_row(
                'FSM_TRANSITION',
                f'{self._prev_state}->{new_state}',
                f'duree_precedent={duration_s}'
            )
        else:
            self._write_row('FSM_TRANSITION', f'INIT->{new_state}')

        self._prev_state = new_state
        self._state_enter_time = now

    def _log_summary(self) -> None:
        self._write_row(
            'SESSION_SUMMARY',
            f'danger={self._danger_count}',
            f'attention={self._attention_count} etat_actuel={self._prev_state}'
        )
        self.get_logger().info(
            f'[METRIQUES] Danger: {self._danger_count} | '
            f'Attention: {self._attention_count} | '
            f'Etat: {self._prev_state}'
        )

    def destroy_node(self) -> None:
        self._log_summary()
        self._csv_file.close()
        self.get_logger().info(f'Metriques sauvegardees : {self._csv_path}')
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetricsRecorderNode()
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
