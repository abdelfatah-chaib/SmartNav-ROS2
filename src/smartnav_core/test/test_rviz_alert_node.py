"""
Tests pour RvizAlertNode (visualisation RViz).

Verifie :
  - Couleur du marqueur canne selon le niveau d'alerte (blanc / orange / rouge)
  - Texte du marqueur selon le niveau d'alerte
  - Oscillation visuelle activee/desactivee selon le niveau de vibration
  - Publication sur deux topics (canne_marker + alert_text)
  - Dedoublonnage des logs d'alerte
"""

import math

import pytest
import rclpy
from smartnav_core.rviz_alert_node import _VIBRATION_AMPLITUDE, RvizAlertNode
from std_msgs.msg import Int8


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class _MockPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _make_node():
    node = RvizAlertNode()
    mock_canne = _MockPublisher()
    mock_text = _MockPublisher()
    node.marker_pub = mock_canne
    node.text_pub = mock_text
    return node, mock_canne, mock_text


def _alert_msg(level: int) -> Int8:
    msg = Int8()
    msg.data = level
    return msg


def _vib_msg(level: int) -> Int8:
    msg = Int8()
    msg.data = level
    return msg


# ── Couleur du marqueur ───────────────────────────────────────────────────────

def test_clear_marker_is_white():
    node, pub_canne, _ = _make_node()
    node.alert_callback(_alert_msg(0))
    node.publish_markers()
    marker = pub_canne.messages[-1]
    assert marker.color.r == pytest.approx(1.0)
    assert marker.color.g == pytest.approx(1.0)
    assert marker.color.b == pytest.approx(1.0)
    node.destroy_node()


def test_warning_marker_is_orange():
    node, pub_canne, _ = _make_node()
    node.alert_callback(_alert_msg(1))
    node.publish_markers()
    marker = pub_canne.messages[-1]
    assert marker.color.r == pytest.approx(1.0)
    assert marker.color.g == pytest.approx(0.5)
    assert marker.color.b == pytest.approx(0.0)
    node.destroy_node()


def test_danger_marker_is_red():
    node, pub_canne, _ = _make_node()
    node.alert_callback(_alert_msg(2))
    node.publish_markers()
    marker = pub_canne.messages[-1]
    assert marker.color.r == pytest.approx(1.0)
    assert marker.color.g == pytest.approx(0.0)
    assert marker.color.b == pytest.approx(0.0)
    node.destroy_node()


# ── Texte flottant ────────────────────────────────────────────────────────────

def test_text_clear():
    node, _, pub_text = _make_node()
    node.alert_callback(_alert_msg(0))
    node.publish_markers()
    assert pub_text.messages[-1].text == 'Voie libre'
    node.destroy_node()


def test_text_warning():
    node, _, pub_text = _make_node()
    node.alert_callback(_alert_msg(1))
    node.publish_markers()
    assert 'ATTENTION' in pub_text.messages[-1].text
    node.destroy_node()


def test_text_danger():
    node, _, pub_text = _make_node()
    node.alert_callback(_alert_msg(2))
    node.publish_markers()
    assert 'DANGER' in pub_text.messages[-1].text
    node.destroy_node()


# ── Vibration visuelle ────────────────────────────────────────────────────────

def test_no_vibration_offset_at_level_0():
    """Niveau vibration 0 -> position y du marqueur doit rester a 0."""
    node, pub_canne, _ = _make_node()
    node._vibration_callback(_vib_msg(0))
    node._phase = 0.0
    node.publish_markers()
    assert pub_canne.messages[-1].pose.position.y == pytest.approx(0.0, abs=1e-6)
    node.destroy_node()


def test_vibration_offset_nonzero_at_level_2():
    """Niveau vibration 2 + phase pi/2 -> decalage y non nul (sin=1, amplitude max)."""
    node, pub_canne, _ = _make_node()
    node._vibration_callback(_vib_msg(2))
    node._phase = math.pi / 2
    node.publish_markers()
    y = pub_canne.messages[-1].pose.position.y
    assert abs(y) > 0.01, f'Decalage y trop faible : {y}'
    node.destroy_node()


def test_vibration_level_1_less_than_level_2():
    """L'amplitude de vibration 1 doit etre inferieure a celle de vibration 2."""
    assert _VIBRATION_AMPLITUDE[1] < _VIBRATION_AMPLITUDE[2]


# ── Dedoublonnage logs ────────────────────────────────────────────────────────

def test_alert_log_deduplication():
    """Le meme niveau d'alerte deux fois de suite ne change _prev_alert_level qu'une fois."""
    node, _, _ = _make_node()
    node.alert_callback(_alert_msg(2))
    assert node._prev_alert_level == 2
    node.alert_callback(_alert_msg(2))
    assert node._prev_alert_level == 2
    node.destroy_node()


# ── Publication double topic ──────────────────────────────────────────────────

def test_both_markers_published():
    """publish_markers() doit toujours publier sur les deux topics."""
    node, pub_canne, pub_text = _make_node()
    node.publish_markers()
    assert len(pub_canne.messages) == 1
    assert len(pub_text.messages) == 1
    node.destroy_node()
