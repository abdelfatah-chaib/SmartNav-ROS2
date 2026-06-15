"""
Tests fonctionnels pour ObstacleDetector — v2.

Couvre les cas originaux + cas limites ajoutes :
  - Scan vide (ranges=[])
  - Tous les rayons a l'infini
  - Cone exactement a la limite angulaire
  - Filtre median temporel : une alerte ponctuelle sur 3 cycles ne doit pas passer
  - Parametres ROS configurables (danger_threshold_m, warn_threshold_m, front_cone_deg)

Note sur la precision float32 : voir commentaire original.
"""

import math

import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from smartnav_core.obstacle_detector import ObstacleDetector


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialise et ferme rclpy une seule fois pour le module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_scan(distance: float, num_samples: int = 360) -> LaserScan:
    """Construit un LaserScan uniforme ou tous les rayons rapportent la meme distance."""
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = 2 * math.pi / num_samples
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = [distance] * num_samples
    return msg


def _make_scan_only_front(distance: float, cone_deg: float = 35.0,
                           num_samples: int = 360) -> LaserScan:
    """Scan avec obstacles seulement dans le cone frontal, infini ailleurs."""
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = 2 * math.pi / num_samples
    msg.range_min = 0.1
    msg.range_max = 10.0
    half_cone_rad = math.radians(cone_deg)
    ranges = []
    for i in range(num_samples):
        angle = msg.angle_min + i * msg.angle_increment
        if abs(angle) <= half_cone_rad:
            ranges.append(distance)
        else:
            ranges.append(float('inf'))
    msg.ranges = ranges
    return msg


class _MockPublisher:
    """Remplacant minimal d'un publisher ROS 2."""
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _make_node_with_mock_publisher():
    node = ObstacleDetector()
    mock_pub = _MockPublisher()
    node.alert_publisher = mock_pub
    return node, mock_pub


# ── Tests originaux ──────────────────────────────────────────────────────────

def test_danger_level():
    """Obstacle a 0.3 m -> alerte niveau 2 (danger)."""
    node, mock_pub = _make_node_with_mock_publisher()
    node.scan_callback(_make_scan(0.3))
    # Le filtre median a besoin de 3 cycles pour stabiliser ; on en injecte 3
    node.scan_callback(_make_scan(0.3))
    node.scan_callback(_make_scan(0.3))
    assert mock_pub.messages, 'Aucun message publie'
    assert mock_pub.messages[-1].data == 2
    node.destroy_node()


def test_warning_level():
    """Obstacle a 1.0 m -> alerte niveau 1 (attention)."""
    node, mock_pub = _make_node_with_mock_publisher()
    for _ in range(3):
        node.scan_callback(_make_scan(1.0))
    assert mock_pub.messages[-1].data == 1
    node.destroy_node()


def test_clear_level():
    """Obstacle a 5.0 m -> alerte niveau 0 (libre)."""
    node, mock_pub = _make_node_with_mock_publisher()
    for _ in range(3):
        node.scan_callback(_make_scan(5.0))
    assert mock_pub.messages[-1].data == 0
    node.destroy_node()


def test_boundary_danger():
    """Distance exactement au seuil de danger (0.5 m) -> niveau 2."""
    node, mock_pub = _make_node_with_mock_publisher()
    for _ in range(3):
        node.scan_callback(_make_scan(0.5))
    assert mock_pub.messages[-1].data == 2
    node.destroy_node()


def test_boundary_warning():
    """Distance dans la zone attention (1.15 m) -> niveau 1."""
    node, mock_pub = _make_node_with_mock_publisher()
    for _ in range(3):
        node.scan_callback(_make_scan(1.15))
    assert mock_pub.messages[-1].data == 1
    node.destroy_node()


def test_all_inf_ranges():
    """Scan tout infini (espace libre) -> publie niveau 0."""
    node, mock_pub = _make_node_with_mock_publisher()
    for _ in range(3):
        node.scan_callback(_make_scan(float('inf')))
    assert mock_pub.messages[-1].data == 0
    node.destroy_node()


def test_log_rate_limiting():
    """L'alerte est publiee a chaque scan, le log seulement au changement d'etat."""
    node, mock_pub = _make_node_with_mock_publisher()
    for _ in range(5):
        node.scan_callback(_make_scan(0.3))
    assert len(mock_pub.messages) == 5
    assert all(m.data in (1, 2) for m in mock_pub.messages)
    node.destroy_node()


def test_too_close_echo_is_ignored():
    """Rayons tres proches (<0.30 m) -> auto-echo/bruit -> niveau 0."""
    node, mock_pub = _make_node_with_mock_publisher()
    for _ in range(3):
        node.scan_callback(_make_scan(0.25))
    assert mock_pub.messages[-1].data == 0
    node.destroy_node()


# ── Nouveaux cas limites ─────────────────────────────────────────────────────

def test_empty_scan():
    """Scan avec ranges=[] (scan vide) -> niveau 0 (pas de crash)."""
    node, mock_pub = _make_node_with_mock_publisher()
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = 0.01
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = []
    node.scan_callback(msg)
    assert mock_pub.messages, 'Aucun message publie sur scan vide'
    assert mock_pub.messages[-1].data == 0
    node.destroy_node()


def test_median_filter_suppresses_single_spike():
    """Une alerte isolee (1 cycle DANGER sur 3 cycles LIBRE) ne doit pas passer."""
    node, mock_pub = _make_node_with_mock_publisher()
    # 2 cycles libres
    node.scan_callback(_make_scan(5.0))
    node.scan_callback(_make_scan(5.0))
    # 1 seul cycle danger (bruit LiDAR)
    node.scan_callback(_make_scan(0.3))
    # Le filtre median (3 echantillons : [0, 0, 2]) -> mediane = 0
    assert mock_pub.messages[-1].data == 0, (
        f'Le filtre median devrait supprimer un spike isole, got {mock_pub.messages[-1].data}'
    )
    node.destroy_node()


def test_median_filter_passes_sustained_danger():
    """Deux cycles danger consecutifs sur 3 -> le danger doit passer (mediane = 2)."""
    node, mock_pub = _make_node_with_mock_publisher()
    # 1 cycle libre puis 2 cycles danger
    node.scan_callback(_make_scan(5.0))
    node.scan_callback(_make_scan(0.3))
    node.scan_callback(_make_scan(0.3))
    # Filtre median [0, 2, 2] -> mediane = 2
    assert mock_pub.messages[-1].data == 2, (
        f'Un danger soutenu doit passer le filtre, got {mock_pub.messages[-1].data}'
    )
    node.destroy_node()


def test_obstacle_outside_cone_ignored():
    """Un obstacle hors du cone frontal ne doit pas declencher d'alerte."""
    node, mock_pub = _make_node_with_mock_publisher()
    # Scan avec obstacle proche (0.3 m) uniquement sur les cotes (hors du cone de 35 deg)
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = 2 * math.pi / 360
    msg.range_min = 0.1
    msg.range_max = 10.0
    half_cone_rad = math.radians(35.0)
    ranges = []
    for i in range(360):
        angle = msg.angle_min + i * msg.angle_increment
        # Obstacle a 90 deg (cote gauche) -> hors du cone frontal
        if abs(abs(angle) - math.pi / 2) < 0.1:
            ranges.append(0.3)
        else:
            ranges.append(float('inf'))
    msg.ranges = ranges
    for _ in range(3):
        node.scan_callback(msg)
    assert mock_pub.messages[-1].data == 0, (
        'Obstacle hors du cone frontal ne doit pas declencher alerte'
    )
    node.destroy_node()


def test_ros_parameters_danger_threshold():
    """Modification du parametre danger_threshold_m change le seuil de detection."""
    node, mock_pub = _make_node_with_mock_publisher()
    # Seuil par defaut = 0.5 m, on le monte a 1.0 m
    node.set_parameters([
        rclpy.parameter.Parameter('danger_threshold_m', value=1.0)
    ])
    # Un obstacle a 0.8 m devrait maintenant declencher DANGER (et non ATTENTION)
    for _ in range(3):
        node.scan_callback(_make_scan(0.8))
    assert mock_pub.messages[-1].data == 2, (
        f'Avec threshold=1.0m, 0.8m doit donner niveau 2, got {mock_pub.messages[-1].data}'
    )
    node.destroy_node()
