#!/usr/bin/env python3
"""
Canne SmartNav — Feedback node (son + vibration).

Abonne /smartnav/alert_level (Int8) et reagit :
  DANGER (2)    -> son d'alarme rapide + vibration forte (/smartnav/vibration = 2)
  ATTENTION (1) -> bip d'avertissement + vibration moderee (/smartnav/vibration = 1)
  LIBRE (0)     -> bip de confirmation + stop vibration (/smartnav/vibration = 0)

Le son est genere en memoire (module wave standard) et joue via aplay
sans aucune dependance externe. Chaque son tourne dans un thread daemon
pour ne pas bloquer la boucle ROS.
"""

from __future__ import annotations

import io
import math
import struct
import subprocess
import threading
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

# ── Parametres audio ──────────────────────────────────────────────────────────
SAMPLE_RATE = 44100

# Danger : bip aigu repete 4x (frequence haute, duree courte)
DANGER_FREQ_HZ = 1200.0
DANGER_BEEP_MS = 150
DANGER_PAUSE_MS = 80
DANGER_REPEAT = 4

# Attention : bip moyen repete 2x
WARN_FREQ_HZ = 800.0
WARN_BEEP_MS = 250
WARN_PAUSE_MS = 150
WARN_REPEAT = 2

# Libre : bip grave court confirmatif
CLEAR_FREQ_HZ = 500.0
CLEAR_BEEP_MS = 200
CLEAR_PAUSE_MS = 0
CLEAR_REPEAT = 1


def _make_tone(freq_hz: float, duration_ms: int, amplitude: float = 0.6) -> bytes:
    """Genere des echantillons PCM 16 bits pour un ton pur."""
    n_samples = int(SAMPLE_RATE * duration_ms / 1000)
    samples = []
    for i in range(n_samples):
        env = 1.0
        fade = int(n_samples * 0.10)
        if i < fade:
            env = i / fade
        elif i > n_samples - fade:
            env = (n_samples - i) / fade
        value = int(
            amplitude * env * 32767
            * math.sin(2.0 * math.pi * freq_hz * i / SAMPLE_RATE)
        )
        samples.append(struct.pack('<h', value))
    return b''.join(samples)


def _make_silence(duration_ms: int) -> bytes:
    n_samples = int(SAMPLE_RATE * duration_ms / 1000)
    return b'\x00\x00' * n_samples


def _build_wav(freq_hz: float, beep_ms: int, pause_ms: int, repeat: int) -> bytes:
    """Construit un fichier WAV en memoire (mono 16 bits)."""
    tone = _make_tone(freq_hz, beep_ms)
    silence = _make_silence(pause_ms) if pause_ms > 0 else b''
    raw_pcm = (tone + silence) * repeat
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(raw_pcm)
    return buf.getvalue()


# Pre-calculer les buffers WAV au demarrage du module
_WAV_DANGER = _build_wav(DANGER_FREQ_HZ, DANGER_BEEP_MS, DANGER_PAUSE_MS, DANGER_REPEAT)
_WAV_WARN = _build_wav(WARN_FREQ_HZ, WARN_BEEP_MS, WARN_PAUSE_MS, WARN_REPEAT)
_WAV_CLEAR = _build_wav(CLEAR_FREQ_HZ, CLEAR_BEEP_MS, CLEAR_PAUSE_MS, CLEAR_REPEAT)

_BEEP_LOCK = threading.Lock()
_active_beep: subprocess.Popen | None = None


def _play_wav(wav_bytes: bytes) -> None:
    """Joue un buffer WAV via aplay dans le thread courant (bloquant)."""
    global _active_beep
    try:
        proc = subprocess.Popen(
            ['aplay', '-q', '-f', 'cd', '--rate', str(SAMPLE_RATE),
             '--channels', '1', '--format', 'S16_LE', '-'],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        with _BEEP_LOCK:
            _active_beep = proc
        proc.communicate(input=wav_bytes)
    except FileNotFoundError:
        print('\a', flush=True)
    except Exception:
        pass
    finally:
        with _BEEP_LOCK:
            _active_beep = None


def _stop_current_beep() -> None:
    """Interrompt immediatement le bip en cours (si actif)."""
    with _BEEP_LOCK:
        proc = _active_beep
    if proc is not None:
        try:
            proc.terminate()
        except Exception:
            pass


def _play_async(wav_bytes: bytes) -> None:
    """Lance la lecture audio dans un thread daemon (non bloquant)."""
    _stop_current_beep()
    t = threading.Thread(target=_play_wav, args=(wav_bytes,), daemon=True)
    t.start()


# ── Noeud ROS 2 ───────────────────────────────────────────────────────────────

class CanneFeedbackNode(Node):
    """Produit son + vibration selon le niveau d'alerte de la canne."""

    def __init__(self) -> None:
        super().__init__('canne_feedback_node')

        self._prev_level: int = -1
        self._lidar_alert_level: int = 0
        self._crossing_alert_level: int = 0

        self._alert_sub = self.create_subscription(
            Int8,
            '/smartnav/alert_level',
            self._on_lidar_alert,
            10,
        )
        self._crossing_alert_sub = self.create_subscription(
            Int8,
            '/smartnav/crossing_alert',
            self._on_crossing_alert,
            10,
        )
        self._vibration_pub = self.create_publisher(Int8, '/smartnav/vibration', 10)

        self._current_vibration: int = 0
        self._vibration_timer = self.create_timer(0.1, self._vibration_heartbeat)

        self.get_logger().info('canne_feedback_node demarre — son et vibration actifs.')

    def _on_lidar_alert(self, msg: Int8) -> None:
        self._lidar_alert_level = int(msg.data)
        self._apply_effective_alert()

    # Backward compatibility for existing tests/helpers that call _on_alert directly.
    def _on_alert(self, msg: Int8) -> None:
        self._on_lidar_alert(msg)

    def _on_crossing_alert(self, msg: Int8) -> None:
        self._crossing_alert_level = int(msg.data)
        self._apply_effective_alert()

    def _apply_effective_alert(self) -> None:
        level = max(self._lidar_alert_level, self._crossing_alert_level)
        if level == self._prev_level:
            return
        prev = self._prev_level
        self._prev_level = level

        if level == 2:
            self.get_logger().error(
                'DANGER — ARRET REQUIS ! Son alarme + vibration forte.'
            )
            _play_async(_WAV_DANGER)
            self._current_vibration = 2

        elif level == 1:
            self.get_logger().warn(
                'ATTENTION — Obstacle detecte. Bip avertissement + vibration moderee.'
            )
            _play_async(_WAV_WARN)
            self._current_vibration = 1

        else:
            self.get_logger().info('VOIE LIBRE — Vibration arretee.')
            # Ne joue le bip de confirmation que si on revient d'un etat d'alerte.
            # Evite le bip parasite au demarrage (transition -1 -> 0).
            if prev > 0:
                _play_async(_WAV_CLEAR)
            self._current_vibration = 0

        vib_msg = Int8()
        vib_msg.data = self._current_vibration
        self._vibration_pub.publish(vib_msg)

    def _vibration_heartbeat(self) -> None:
        """Publie l'etat de vibration en continu (10 Hz) pour les abonnes tardifs."""
        vib_msg = Int8()
        vib_msg.data = self._current_vibration
        self._vibration_pub.publish(vib_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanneFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _stop_current_beep()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
