"""
Tests pour SmartNavNavigator — FSM 4 etats.

Verifie :
  - WANDER -> DANGER_HOLD sur alert_level=2 (voiture)
  - DANGER_HOLD -> WANDER apres DANGER_HOLD_CLEAR_CYCLES cycles alert_level=0
  - DANGER_HOLD reste en place si alert_level != 0
  - WANDER -> OBSTACLE_DETECTED quand front_min < OBSTACLE_DISTANCE_M
  - OBSTACLE_DETECTED -> AVOIDANCE immediat
  - AVOIDANCE -> WANDER apres CLEAR_CONFIRM_CYCLES cycles + MIN_AVOIDANCE_TIME_S
  - AVOIDANCE -> DANGER_HOLD si alert_level=2 survient pendant l'evitement
  - Mode non-autonome : stop publie, pas de transition d'etat
  - Watchdog LiDAR : stop si pas de scan recent
  - Choix du sens de rotation : tourne vers le cote le plus degage
"""

from geometry_msgs.msg import Twist  # noqa: F401
import pytest
import rclpy
from smartnav_core.smartnav_navigator import (
    CLEAR_CONFIRM_CYCLES,
    CLEAR_DISTANCE_M,
    DANGER_HOLD_CLEAR_CYCLES,
    MIN_AVOIDANCE_TIME_S,
    NavState,
    OBSTACLE_DISTANCE_M,
    SmartNavNavigator,
)
from std_msgs.msg import Int8, String


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


# ── Helpers ───────────────────────────────────────────────────────────────────

class _MockPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _make_node() -> SmartNavNavigator:
    node = SmartNavNavigator()
    node._cmd_pub = _MockPublisher()
    node._node_start_time = (
        node.get_clock().now() - rclpy.duration.Duration(seconds=100)
    )
    return node


def _inject_scan(node: SmartNavNavigator, front: float = 5.0,
                 left: float = 5.0, right: float = 5.0) -> None:
    with node._lock:
        node._front_min = front
        node._front_half_min = front
        node._left_min = left
        node._right_min = right
        node._last_scan_stamp = node.get_clock().now()


def _inject_alert(node: SmartNavNavigator, level: int) -> None:
    msg = Int8()
    msg.data = level
    node._on_alert(msg)


def _run_control(node: SmartNavNavigator, cycles: int = 1) -> None:
    for _ in range(cycles):
        node._control_loop()


def _last_cmd(node: SmartNavNavigator):
    return node._cmd_pub.messages[-1]


def _is_stopped(node: SmartNavNavigator) -> bool:
    cmd = _last_cmd(node)
    return (cmd.linear.x == pytest.approx(0.0)
            and cmd.angular.z == pytest.approx(0.0))


# ── Tests FSM ─────────────────────────────────────────────────────────────────

def test_initial_state_is_wander():
    node = _make_node()
    assert node._state == NavState.WANDER
    node.destroy_node()


def test_wander_publishes_forward():
    node = _make_node()
    _inject_scan(node, front=5.0)
    _inject_alert(node, 0)
    _run_control(node)
    cmd = _last_cmd(node)
    assert cmd.linear.x > 0.0, 'WANDER devrait avancer'
    assert cmd.angular.z == pytest.approx(0.0)
    node.destroy_node()


def test_danger_alert_triggers_danger_hold():
    """alert_level=2 depuis WANDER -> DANGER_HOLD immediatement."""
    node = _make_node()
    _inject_scan(node, front=5.0)
    _inject_alert(node, 2)
    _run_control(node)
    assert node._state == NavState.DANGER_HOLD, (
        f'Attendu DANGER_HOLD, obtenu {node._state}'
    )
    node.destroy_node()


def test_danger_hold_publishes_stop():
    node = _make_node()
    _inject_scan(node, front=5.0)
    _inject_alert(node, 2)
    _run_control(node)
    assert _is_stopped(node), 'DANGER_HOLD doit publier stop (0,0)'
    node.destroy_node()


def test_danger_hold_stays_while_alert_persists():
    """DANGER_HOLD doit rester tant que alert_level != 0."""
    node = _make_node()
    _inject_scan(node, front=5.0)
    _inject_alert(node, 2)
    _run_control(node)
    _inject_alert(node, 2)
    _run_control(node, cycles=20)
    assert node._state == NavState.DANGER_HOLD
    node.destroy_node()


def test_danger_hold_exits_after_clear_cycles():
    """DANGER_HOLD -> WANDER apres DANGER_HOLD_CLEAR_CYCLES cycles a alert=0."""
    node = _make_node()
    _inject_scan(node, front=5.0)
    _inject_alert(node, 2)
    _run_control(node)

    _inject_alert(node, 0)
    _run_control(node, cycles=DANGER_HOLD_CLEAR_CYCLES)
    assert node._state == NavState.WANDER, (
        f'Attendu WANDER apres {DANGER_HOLD_CLEAR_CYCLES} cycles libres, '
        f'obtenu {node._state}'
    )
    node.destroy_node()


def test_danger_hold_counter_resets_on_alert():
    """Si l'alerte revient pendant DANGER_HOLD, le compteur se remet a zero."""
    node = _make_node()
    _inject_scan(node, front=5.0)
    _inject_alert(node, 2)
    _run_control(node)

    _inject_alert(node, 0)
    _run_control(node, cycles=DANGER_HOLD_CLEAR_CYCLES - 2)
    assert node._state == NavState.DANGER_HOLD

    _inject_alert(node, 2)
    _run_control(node)
    assert node._danger_clear_cycles == 0
    node.destroy_node()


def test_static_obstacle_triggers_avoidance():
    """Obstacle statique (LiDAR) -> OBSTACLE_DETECTED -> AVOIDANCE."""
    node = _make_node()
    _inject_alert(node, 0)
    _inject_scan(node, front=OBSTACLE_DISTANCE_M - 0.1)
    _run_control(node)
    assert node._state == NavState.OBSTACLE_DETECTED
    _run_control(node)
    assert node._state == NavState.AVOIDANCE
    node.destroy_node()


def test_avoidance_turns_not_forward():
    """En AVOIDANCE, la commande doit avoir angular_z != 0 et linear_x == 0."""
    node = _make_node()
    _inject_alert(node, 0)
    _inject_scan(node, front=OBSTACLE_DISTANCE_M - 0.1)
    _run_control(node, 2)
    _inject_scan(node, front=OBSTACLE_DISTANCE_M - 0.1)
    _run_control(node)
    cmd = _last_cmd(node)
    assert cmd.linear.x == pytest.approx(0.0), 'AVOIDANCE ne doit pas avancer'
    assert abs(cmd.angular.z) > 0.0, 'AVOIDANCE doit tourner'
    node.destroy_node()


def test_avoidance_exits_when_clear():
    """AVOIDANCE -> WANDER apres CLEAR_CONFIRM_CYCLES cycles degage + MIN_AVOIDANCE_TIME_S."""
    node = _make_node()
    _inject_alert(node, 0)
    _inject_scan(node, front=OBSTACLE_DISTANCE_M - 0.1)
    _run_control(node, 2)

    node._state_enter_time = (
        node.get_clock().now() - rclpy.duration.Duration(seconds=MIN_AVOIDANCE_TIME_S + 0.1)
    )

    _inject_scan(node, front=CLEAR_DISTANCE_M + 0.5)
    _run_control(node, cycles=CLEAR_CONFIRM_CYCLES)
    assert node._state == NavState.WANDER, f'Attendu WANDER, obtenu {node._state}'
    node.destroy_node()


def test_avoidance_interrupted_by_danger():
    """Si alert=2 survient pendant AVOIDANCE, passer en DANGER_HOLD."""
    node = _make_node()
    _inject_alert(node, 0)
    _inject_scan(node, front=OBSTACLE_DISTANCE_M - 0.1)
    _run_control(node, 2)
    _inject_alert(node, 2)
    _run_control(node)
    assert node._state == NavState.DANGER_HOLD
    node.destroy_node()


def test_turn_direction_toward_more_open_side():
    """Le navigateur doit tourner vers le cote le plus degage."""
    node = _make_node()
    _inject_alert(node, 0)
    _inject_scan(node, front=OBSTACLE_DISTANCE_M - 0.1, left=4.0, right=0.8)
    _run_control(node, 2)
    _inject_scan(node, front=OBSTACLE_DISTANCE_M - 0.1, left=4.0, right=0.8)
    _run_control(node)
    cmd = _last_cmd(node)
    assert cmd.angular.z > 0.0, f'Devrait tourner a gauche, angular_z={cmd.angular.z}'
    node.destroy_node()


def test_non_autonomous_mode_stops():
    """En mode non-autonome, le noeud publie stop et ne change pas d'etat."""
    node = _make_node()
    mode_msg = String()
    mode_msg.data = 'manual'
    node._on_mode(mode_msg)
    _inject_scan(node, front=5.0)
    _inject_alert(node, 0)
    _run_control(node)
    assert _is_stopped(node)
    node.destroy_node()


def test_watchdog_stops_if_no_scan():
    """Si le scan est trop vieux (> WATCHDOG_TIMEOUT_S), le noeud publie stop."""
    node = _make_node()
    with node._lock:
        node._last_scan_stamp = (
            node.get_clock().now() - rclpy.duration.Duration(seconds=2.0)
        )
    _run_control(node)
    assert _is_stopped(node), 'Watchdog doit stopper le robot'
    node.destroy_node()
