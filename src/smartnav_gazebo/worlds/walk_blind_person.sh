#!/usr/bin/env bash
# Continuous trajectory controller for blind_person in smartnav_world.
# The person crosses the street, turns, and continues walking on sidewalks.

set -euo pipefail

WORLD="${1:-smartnav_world}"

python3 - "$WORLD" <<'PY'
import fcntl
import math
import re
import subprocess
import sys
import threading
import time

WORLD = sys.argv[1]
MODEL = "blind_person"
CAR_MODELS = ("moving_car_A", "moving_car_B")
GZ_LOCK_PATH = "/tmp/smartnav_gz_transport.lock"

# Motion parameters.
DT = 0.10
LINEAR_SPEED_MPS = 1.60
CROSSING_SPEED_MPS = 1.80
YAW_RATE_RADPS = 1.20

# Safety: wait at curb if a car arrives within this window.
CROSSING_X = 3.0
# Measured in Gazebo (~5.2 m/s); must match actual car speed for ETA safety.
CAR_SPEED_MPS = 5.2
UNSAFE_ETA_S = 7.0

SAFETY_CHECK_DT = 0.30

# Single-worker pose thread: only one gz service call at a time (avoids concurrent
# subprocess flood that causes "Host unreachable" in Gazebo's transport layer).
# The worker always picks the LATEST requested pose, skipping intermediate ones.
_next_req: list = [None]
_req_lock = threading.Lock()
_wake = threading.Event()


def _run_gz(args: list, timeout: float = 2.0) -> subprocess.CompletedProcess:
    with open(GZ_LOCK_PATH, "w", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file, fcntl.LOCK_EX)
        return subprocess.run(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )


def _pose_worker() -> None:
    while True:
        _wake.wait(timeout=1.0)
        _wake.clear()
        with _req_lock:
            req = _next_req[0]
            _next_req[0] = None
        if not req:
            continue
        _run_gz(
            [
                "gz", "service",
                "-s", f"/world/{WORLD}/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "200",
                "--req", req,
            ],
            timeout=3.0,
        )


_worker = threading.Thread(target=_pose_worker, daemon=True)
_worker.start()


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def ground_height(y: float) -> float:
    ay = abs(y)
    road_z = 0.08
    sidewalk_z = 0.20
    ramp_start = 3.7
    ramp_end = 4.3
    if ay <= ramp_start:
        return road_z
    if ay >= ramp_end:
        return sidewalk_z
    t = (ay - ramp_start) / (ramp_end - ramp_start)
    smooth = t * t * (3.0 - 2.0 * t)
    return road_z + (sidewalk_z - road_z) * smooth


def set_pose(x: float, y: float, yaw: float) -> None:
    """Queue a pose update for the single background worker.

    The main timing loop is never blocked: it writes the latest pose and
    signals the worker.  If the worker is still processing the previous
    call it will pick up the latest value after it finishes (skipping
    intermediate positions), preventing both queue build-up and the
    'Host unreachable' flood that concurrent subprocesses cause.
    """
    z = ground_height(y)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = (
        f"name: '{MODEL}' "
        f"position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }} "
        f"orientation {{ x: 0.0 y: 0.0 z: {qz:.6f} w: {qw:.6f} }}"
    )
    with _req_lock:
        _next_req[0] = req
    _wake.set()


def get_models_xy(model_names: tuple) -> dict:
    try:
        result = _run_gz(
            [
                "gz", "topic",
                "-e", "-n", "1",
                "-t", f"/world/{WORLD}/pose/info",
            ],
            timeout=1.0,
        )
    except subprocess.TimeoutExpired:
        return {}
    if result.returncode != 0 or not result.stdout:
        return {}

    poses = {}
    for name in model_names:
        pattern = (
            rf'name:\s*"{re.escape(name)}".*?'
            r"position\s*\{\s*x:\s*([-+]?\d*\.?\d+)\s*y:\s*([-+]?\d*\.?\d+)"
        )
        m = re.search(pattern, result.stdout, flags=re.S)
        if m:
            poses[name] = (float(m.group(1)), float(m.group(2)))
    return poses


def crossing_eta_is_unsafe(poses: dict) -> bool:
    car_a = poses.get("moving_car_A")
    car_b = poses.get("moving_car_B")
    if car_a is None or car_b is None:
        return False
    eta_a = (CROSSING_X - car_a[0]) / CAR_SPEED_MPS
    eta_b = (car_b[0] - CROSSING_X) / CAR_SPEED_MPS
    return (0.0 <= eta_a <= UNSAFE_ETA_S) or (0.0 <= eta_b <= UNSAFE_ETA_S)


def wait_until_crossing_safe(px: float, py: float) -> None:
    while True:
        poses = get_models_xy(CAR_MODELS)
        if not crossing_eta_is_unsafe(poses):
            break
        distances = [math.hypot(cx - px, cy - py) for cx, cy in poses.values()]
        dist = min(distances) if distances else None
        if dist is not None:
            print(
                f"[blind_person] HOLD crossing, nearest car {dist:.1f}m",
                flush=True,
            )
        time.sleep(SAFETY_CHECK_DT)
    print("[blind_person] RESUME crossing — voie libre", flush=True)


def move_straight(
    x: float, y: float, yaw: float,
    target_x: float, target_y: float,
    speed_mps: float = LINEAR_SPEED_MPS,
) -> tuple:
    distance = math.hypot(target_x - x, target_y - y)
    steps = max(1, int(distance / (speed_mps * DT)))
    t0 = time.monotonic()
    for i in range(1, steps + 1):
        t = i / steps
        px = x + (target_x - x) * t
        py = y + (target_y - y) * t
        set_pose(px, py, yaw)
        # Sleep until the next scheduled tick (drift-compensated).
        deadline = t0 + i * DT
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    return target_x, target_y, yaw


def turn_in_place(x: float, y: float, yaw: float, target_yaw: float) -> tuple:
    delta = normalize_angle(target_yaw - yaw)
    duration = abs(delta) / YAW_RATE_RADPS
    steps = max(1, int(duration / DT))
    t0 = time.monotonic()
    for i in range(1, steps + 1):
        t = i / steps
        pyaw = normalize_angle(yaw + delta * t)
        set_pose(x, y, pyaw)
        deadline = t0 + i * DT
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    return x, y, normalize_angle(target_yaw)


# Rectangular route:
# 1) Cross north → south (crossing segment — safety check applied)
# 2) Turn east, walk south sidewalk
# 3) Turn north, walk north sidewalk
# 4) Turn west, return near crosswalk
# 5) Turn south, repeat
segments = [
    ("move", 3.0, -4.3, -math.pi / 2),
    ("turn", 0.0, 0.0, 0.0),
    ("move", 24.0, -4.3, 0.0),
    ("turn", 0.0, 0.0, math.pi / 2),
    ("move", 24.0, 4.3, math.pi / 2),
    ("turn", 0.0, 0.0, math.pi),
    ("move", 3.0, 4.3, math.pi),
    ("turn", 0.0, 0.0, -math.pi / 2),
    ("move", 3.0, 4.3, -math.pi / 2),
]

x, y, yaw = 3.0, 4.3, -math.pi / 2

# Short wait for Gazebo world to be fully loaded.
time.sleep(4.0)

while True:
    for kind, tx, ty, tyaw in segments:
        if kind == "move":
            is_crossing = (
                math.isclose(x, 3.0, abs_tol=0.15)
                and math.isclose(y, 4.3, abs_tol=0.15)
                and math.isclose(tx, 3.0, abs_tol=0.15)
                and ty < y
            )
            if is_crossing:
                wait_until_crossing_safe(x, y)
                x, y, yaw = move_straight(x, y, yaw, tx, ty,
                                          speed_mps=CROSSING_SPEED_MPS)
            else:
                x, y, yaw = move_straight(x, y, yaw, tx, ty)
        else:
            x, y, yaw = turn_in_place(x, y, yaw, tyaw)
PY
