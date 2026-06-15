#!/usr/bin/env python3
"""
Position-based loop controller for moving_car_A / moving_car_B.

Each car is teleported back to its start pose only after it reaches the far
end of the road (|x| >= END_X).  A background worker drains a request queue so
simultaneous end-of-road events never drop one car's reset.
"""

from __future__ import annotations

import fcntl
import json
import re
import subprocess
import sys
import threading
import time
from collections import deque

WORLD = sys.argv[1] if len(sys.argv) > 1 else "smartnav_world"
POSE_TOPIC = f"/world/{WORLD}/pose/info"
SET_POSE_SVC = f"/world/{WORLD}/set_pose"
DEBUG_LOG = "/home/abdelfatah/smartnav_ws/.cursor/debug-7343fe.log"

START_X = 58.0
END_X = 55.0
OFFROAD_X = 61.0
POLL_DT = 0.4
COOLDOWN_S = 2.0
STARTUP_DELAY_S = 1.0
SET_POSE_RETRIES = 3

CAR_A = "moving_car_A"
CAR_B = "moving_car_B"

LOCK_PATH = "/tmp/smartnav_gz_transport.lock"

_pending: deque[tuple[str, str, float, float, float, float]] = deque()
_pending_lock = threading.Lock()
_wake = threading.Event()


def _debug(message: str, data: dict, hypothesis_id: str) -> None:
    # #region agent log
    try:
        payload = {
            "sessionId": "7343fe",
            "runId": "post-fix-v2",
            "hypothesisId": hypothesis_id,
            "location": "reset_cars_loop.py",
            "message": message,
            "data": data,
            "timestamp": int(time.time() * 1000),
        }
        with open(DEBUG_LOG, "a", encoding="utf-8") as log_file:
            log_file.write(json.dumps(payload) + "\n")
    except OSError:
        pass
    # #endregion


def _run_gz(args: list[str], timeout: float = 2.0) -> subprocess.CompletedProcess[str]:
    with open(LOCK_PATH, "w", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file, fcntl.LOCK_EX)
        return subprocess.run(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )


def _set_pose(req: str) -> bool:
    for attempt in range(SET_POSE_RETRIES):
        result = _run_gz(
            [
                "gz",
                "service",
                "-s",
                SET_POSE_SVC,
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "1000",
                "--req",
                req,
            ],
            timeout=3.0,
        )
        ok = result.returncode == 0 and "Segmentation" not in (result.stderr or "")
        _debug(
            "set_pose_result",
            {
                "attempt": attempt + 1,
                "returncode": result.returncode,
                "stderr": (result.stderr or "")[:120],
                "ok": ok,
            },
            "H2",
        )
        if ok:
            return True
        time.sleep(0.15 * (attempt + 1))
    return False


def _pose_worker() -> None:
    while True:
        _wake.wait(timeout=1.0)
        _wake.clear()

        while True:
            with _pending_lock:
                if not _pending:
                    break
                model, req, x, y, qz, qw = _pending.popleft()

            ok = _set_pose(req)
            _debug(
                "reset_applied",
                {"model": model, "x": x, "y": y, "ok": ok},
                "H2",
            )


_worker = threading.Thread(target=_pose_worker, daemon=True)
_worker.start()


def queue_reset(model: str, x: float, y: float, qz: float, qw: float) -> None:
    req = (
        f"name: '{model}' "
        f"position {{ x: {x:.4f} y: {y:.4f} z: 0.54 }} "
        f"orientation {{ x: 0.0 y: 0.0 z: {qz:.6f} w: {qw:.6f} }}"
    )
    with _pending_lock:
        _pending.append((model, req, x, y, qz, qw))
        queue_len = len(_pending)
    _wake.set()
    _debug("reset_queued", {"model": model, "x": x, "queue_len": queue_len}, "H1")


def get_car_x() -> dict[str, float]:
    try:
        result = _run_gz(["gz", "topic", "-e", "-n", "1", "-t", POSE_TOPIC], timeout=2.0)
    except subprocess.TimeoutExpired:
        return {}
    if result.returncode != 0 or not result.stdout:
        return {}

    positions: dict[str, float] = {}
    for name in (CAR_A, CAR_B):
        pattern = (
            rf'name:\s*"{re.escape(name)}".*?'
            r"position\s*\{\s*x:\s*([-+]?\d*\.?\d+)"
        )
        match = re.search(pattern, result.stdout, flags=re.S)
        if match:
            positions[name] = float(match.group(1))
    return positions


def _car_a_needs_reset(ax: float) -> tuple[bool, bool]:
    """Return (needs_reset, urgent_offroad)."""
    urgent = ax > OFFROAD_X or ax < -(START_X + 2.0)
    normal = ax >= END_X
    return urgent or normal, urgent


def _car_b_needs_reset(bx: float) -> tuple[bool, bool]:
    urgent = bx < -OFFROAD_X or bx > START_X + 2.0
    normal = bx <= -END_X
    return urgent or normal, urgent


def _wait_for_startup_positions() -> None:
    """Block until both cars are near their start poses (or timeout)."""
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        positions = get_car_x()
        ax = positions.get(CAR_A)
        bx = positions.get(CAR_B)
        if ax is not None and bx is not None:
            if abs(ax + START_X) < 4.0 and abs(bx - START_X) < 4.0:
                _debug(
                    "startup_ok",
                    {"A": ax, "B": bx},
                    "H4",
                )
                return
        time.sleep(0.25)
    _debug("startup_timeout", get_car_x(), "H4")


def main() -> None:
    print("╔══════════════════════════════════════════════════════╗")
    print("║   SmartNav — Bouclage des voitures (base position)   ║")
    print(f"║   Monde  : {WORLD:<40}║")
    print(f"║   Bouclage a |x| >= {END_X} m  │  Ctrl+C pour arreter   ║")
    print("╚══════════════════════════════════════════════════════╝")
    print("")
    print(f"⏳ Attente {STARTUP_DELAY_S:.0f} s pour que Gazebo soit pret...")
    time.sleep(STARTUP_DELAY_S)

    cooldown_until: dict[str, float] = {CAR_A: 0.0, CAR_B: 0.0}

    # Always place both cars at known starts (recovers stale sessions / failed resets).
    print("Initialisation des voitures aux positions de depart...", flush=True)
    queue_reset(CAR_A, -START_X, 2.0, 0.0, 1.0)
    queue_reset(CAR_B, START_X, -2.0, 1.0, 0.0)
    _wait_for_startup_positions()
    now = time.monotonic()
    cooldown_until[CAR_A] = now + COOLDOWN_S
    cooldown_until[CAR_B] = now + COOLDOWN_S
    _debug("startup_init", {"A": -START_X, "B": START_X}, "H4")

    while True:
        now = time.monotonic()
        positions = get_car_x()
        ax = positions.get(CAR_A)
        bx = positions.get(CAR_B)

        _debug(
            "poll",
            {"A": ax, "B": bx, "cooldown_A": cooldown_until[CAR_A] - now,
             "cooldown_B": cooldown_until[CAR_B] - now},
            "H3",
        )

        needs_a, urgent_a = (False, False) if ax is None else _car_a_needs_reset(ax)
        if needs_a and (urgent_a or now >= cooldown_until[CAR_A]):
            queue_reset(CAR_A, -START_X, 2.0, 0.0, 1.0)
            cooldown_until[CAR_A] = now + COOLDOWN_S
            ts = time.strftime("%H:%M:%S")
            reason = (
                "recuperation hors route"
                if urgent_a and (ax > OFFROAD_X or ax < -(START_X + 2.0))
                else "trajet complet"
            )
            print(
                f"{ts}  ↺  {CAR_A} → x=-{START_X:.0f}  ({reason}, relance W→E)",
                flush=True,
            )

        needs_b, urgent_b = (False, False) if bx is None else _car_b_needs_reset(bx)
        if needs_b and (urgent_b or now >= cooldown_until[CAR_B]):
            queue_reset(CAR_B, START_X, -2.0, 1.0, 0.0)
            cooldown_until[CAR_B] = now + COOLDOWN_S
            ts = time.strftime("%H:%M:%S")
            reason = (
                "recuperation hors route"
                if urgent_b and (bx < -OFFROAD_X or bx > START_X + 2.0)
                else "trajet complet"
            )
            print(
                f"{ts}  ↺  {CAR_B} → x=+{START_X:.0f}  ({reason}, relance E→W)",
                flush=True,
            )

        time.sleep(POLL_DT)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
