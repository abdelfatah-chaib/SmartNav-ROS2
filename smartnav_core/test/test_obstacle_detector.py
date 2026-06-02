"""
Functional tests for the ObstacleDetector node.

These tests inject a mock LaserScan and verify that the published
alert level (Int8) matches the expected thresholds:
  - distance <= 0.5 m  -> level 2 (danger)
  - distance <= 1.2 m  -> level 1 (warning)
  - distance  > 1.2 m  -> level 0 (clear)
  - distance < 0.30 m  -> ignored (self-echo/noise rejection)

Note on float32 precision: LaserScan.ranges stores float32 values.
Python floats (float64) assigned to LaserScan.ranges may be rounded
to the nearest float32, so exact boundary values (e.g. exactly 1.2)
can resolve to a slightly higher float32, shifting the alert level.
Boundary tests therefore use values safely inside each zone (e.g. 1.15
instead of exactly 1.2).
"""

import math

import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from smartnav_core.obstacle_detector import ObstacleDetector


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialise and shut down rclpy once for the whole test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_scan(distance: float, num_samples: int = 360) -> LaserScan:
    """Build a uniform LaserScan where every ray reports the same distance."""
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = 2 * math.pi / num_samples
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = [distance] * num_samples
    return msg


class _MockPublisher:
    """Minimal stand-in for a ROS 2 publisher that records published messages."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _make_node_with_mock_publisher():
    node = ObstacleDetector()
    mock_pub = _MockPublisher()
    node.alert_publisher = mock_pub
    return node, mock_pub


def test_danger_level():
    """An obstacle at 0.3 m must trigger alert level 2 (danger)."""
    node, mock_pub = _make_node_with_mock_publisher()
    node.scan_callback(_make_scan(0.3))
    assert mock_pub.messages, 'No message published'
    assert mock_pub.messages[-1].data == 2, (
        f'Expected level 2, got {mock_pub.messages[-1].data}'
    )
    node.destroy_node()


def test_warning_level():
    """An obstacle at 1.0 m must trigger alert level 1 (warning)."""
    node, mock_pub = _make_node_with_mock_publisher()
    node.scan_callback(_make_scan(1.0))
    assert mock_pub.messages, 'No message published'
    assert mock_pub.messages[-1].data == 1, (
        f'Expected level 1, got {mock_pub.messages[-1].data}'
    )
    node.destroy_node()


def test_clear_level():
    """An obstacle at 5.0 m must result in alert level 0 (clear)."""
    node, mock_pub = _make_node_with_mock_publisher()
    node.scan_callback(_make_scan(5.0))
    assert mock_pub.messages, 'No message published'
    assert mock_pub.messages[-1].data == 0, (
        f'Expected level 0, got {mock_pub.messages[-1].data}'
    )
    node.destroy_node()


def test_boundary_danger():
    """Distance exactly at the danger threshold (0.5 m) must give level 2."""
    node, mock_pub = _make_node_with_mock_publisher()
    node.scan_callback(_make_scan(0.5))
    assert mock_pub.messages[-1].data == 2
    node.destroy_node()


def test_boundary_warning():
    """Distance safely inside the warning zone (1.15 m) must give level 1."""
    node, mock_pub = _make_node_with_mock_publisher()
    node.scan_callback(_make_scan(1.15))
    assert mock_pub.messages[-1].data == 1
    node.destroy_node()


def test_all_inf_ranges():
    """All-inf scan (open space) must publish level 0."""
    node, mock_pub = _make_node_with_mock_publisher()
    node.scan_callback(_make_scan(float('inf')))
    assert mock_pub.messages[-1].data == 0
    node.destroy_node()


def test_log_rate_limiting():
    """Alert is published every scan but the log fires only on state change."""
    node, mock_pub = _make_node_with_mock_publisher()
    for _ in range(5):
        node.scan_callback(_make_scan(0.3))
    assert len(mock_pub.messages) == 5
    assert all(m.data == 2 for m in mock_pub.messages)
    node.destroy_node()


def test_too_close_echo_is_ignored():
    """Very short ranges (<0.30 m) are treated as self-echo/noise and ignored."""
    node, mock_pub = _make_node_with_mock_publisher()
    node.scan_callback(_make_scan(0.25))
    assert mock_pub.messages, 'No message published'
    assert mock_pub.messages[-1].data == 0
    node.destroy_node()
