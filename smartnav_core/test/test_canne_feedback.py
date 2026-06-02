"""
Tests for CanneFeedbackNode (son + vibration).

Verifie :
  - Publication vibration 2 sur alert_level 2
  - Publication vibration 1 sur alert_level 1
  - Publication vibration 0 sur alert_level 0
  - Dedoublonnage : pas de re-publication si le niveau ne change pas
  - Generation WAV : buffers non-vides, taille proportionnelle a la duree
  - Audio : fallback silencieux si aplay absent (pas d'exception)
"""

import io
import threading
import wave

import pytest
import rclpy
from smartnav_core.canne_feedback_node import (
    _build_wav,
    _play_async,
    _WAV_CLEAR,
    _WAV_DANGER,
    _WAV_WARN,
    CanneFeedbackNode,
    SAMPLE_RATE,
)
from std_msgs.msg import Int8


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


# ── Helpers ──────────────────────────────────────────────────────────────────

class _MockPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _make_node():
    node = CanneFeedbackNode()
    mock_pub = _MockPublisher()
    node._vibration_pub = mock_pub
    return node, mock_pub


def _alert_msg(level: int) -> Int8:
    msg = Int8()
    msg.data = level
    return msg


# ── Tests niveau d'alerte -> vibration ───────────────────────────────────────

def test_danger_sets_vibration_2():
    node, pub = _make_node()
    node._on_alert(_alert_msg(2))
    assert pub.messages, 'Aucun message publie'
    assert pub.messages[-1].data == 2, f'Attendu 2, obtenu {pub.messages[-1].data}'
    node.destroy_node()


def test_warning_sets_vibration_1():
    node, pub = _make_node()
    node._on_alert(_alert_msg(1))
    assert pub.messages, 'Aucun message publie'
    assert pub.messages[-1].data == 1
    node.destroy_node()


def test_clear_sets_vibration_0():
    node, pub = _make_node()
    node._on_alert(_alert_msg(0))
    assert pub.messages, 'Aucun message publie'
    assert pub.messages[-1].data == 0
    node.destroy_node()


def test_no_duplicate_publish_same_level():
    """Si le niveau ne change pas, _on_alert ne publie pas une 2e fois."""
    node, pub = _make_node()
    node._on_alert(_alert_msg(2))
    count_after_first = len(pub.messages)
    node._on_alert(_alert_msg(2))
    assert len(pub.messages) == count_after_first, (
        'Publication dupliquee malgre niveau inchange'
    )
    node.destroy_node()


def test_transition_sequence():
    """Sequence 0->2->1->0 : vibration doit suivre exactement."""
    node, pub = _make_node()
    for level in (0, 2, 1, 0):
        node._prev_level = -1
        node._on_alert(_alert_msg(level))
    values = [m.data for m in pub.messages]
    assert values == [0, 2, 1, 0], f'Sequence inattendue : {values}'
    node.destroy_node()


def test_current_vibration_state_updated():
    """_current_vibration reflete toujours le dernier niveau."""
    node, _ = _make_node()
    node._on_alert(_alert_msg(2))
    assert node._current_vibration == 2
    node._prev_level = -1
    node._on_alert(_alert_msg(0))
    assert node._current_vibration == 0
    node.destroy_node()


# ── Tests generation WAV ──────────────────────────────────────────────────────

def test_wav_buffers_not_empty():
    assert len(_WAV_DANGER) > 0
    assert len(_WAV_WARN) > 0
    assert len(_WAV_CLEAR) > 0


def test_wav_buffers_are_valid_wav():
    """Chaque buffer doit etre un WAV mono 16 bits parseable."""
    for label, buf in [('DANGER', _WAV_DANGER), ('WARN', _WAV_WARN), ('CLEAR', _WAV_CLEAR)]:
        bio = io.BytesIO(buf)
        with wave.open(bio, 'rb') as wf:
            assert wf.getnchannels() == 1, f'{label}: attendu mono'
            assert wf.getsampwidth() == 2, f'{label}: attendu 16 bits'
            assert wf.getframerate() == SAMPLE_RATE, (
                f"{label}: taux d'echant. incorrect"
            )
            assert wf.getnframes() > 0, f'{label}: aucune frame audio'


def test_danger_wav_longer_than_clear():
    """DANGER (4x bips) doit etre plus long que CLEAR (1x bip)."""
    def wav_duration_ms(buf):
        bio = io.BytesIO(buf)
        with wave.open(bio, 'rb') as wf:
            return 1000 * wf.getnframes() / wf.getframerate()
    assert wav_duration_ms(_WAV_DANGER) > wav_duration_ms(_WAV_CLEAR)


def test_build_wav_custom():
    """_build_wav doit produire un WAV dont la duree correspond aux parametres."""
    buf = _build_wav(freq_hz=440.0, beep_ms=100, pause_ms=50, repeat=2)
    bio = io.BytesIO(buf)
    with wave.open(bio, 'rb') as wf:
        duration_ms = 1000 * wf.getnframes() / wf.getframerate()
    assert abs(duration_ms - 300) < 5, f'Duree inattendue : {duration_ms:.1f} ms'


# ── Test audio non-bloquant ───────────────────────────────────────────────────

def test_play_async_does_not_block():
    """_play_async doit retourner immediatement (thread daemon)."""
    import time
    start = time.monotonic()
    _play_async(_WAV_CLEAR)
    elapsed = time.monotonic() - start
    assert elapsed < 0.2, f'_play_async a bloque {elapsed:.3f} s'


def test_play_async_no_exception_without_aplay(monkeypatch):
    """Si aplay est absent, _play_async ne doit pas lever d'exception."""
    import smartnav_core.canne_feedback_node as fb
    original = fb.subprocess.Popen

    def _raise(*args, **kwargs):
        raise FileNotFoundError('aplay not found')

    monkeypatch.setattr(fb.subprocess, 'Popen', _raise)
    try:
        _play_async(_WAV_DANGER)
    except Exception as e:
        pytest.fail(f'Exception inattendue : {e}')
    finally:
        monkeypatch.setattr(fb.subprocess, 'Popen', original)


def test_play_async_uses_daemon_thread():
    """Le thread audio doit etre daemon pour ne pas bloquer le shutdown."""
    threads_before = {t.ident for t in threading.enumerate()}
    _play_async(_WAV_CLEAR)
    new_threads = [
        t for t in threading.enumerate()
        if t.ident not in threads_before
    ]
    assert all(t.daemon for t in new_threads), 'Thread audio non daemon detecte'
