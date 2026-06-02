#!/bin/bash
# ============================================================
#  reset_cars.sh — Bouclage des voitures dynamiques SmartNav
#  Usage : bash reset_cars.sh [world_name]
#  Lancer automatiquement par start_simulation.launch.py
# ============================================================
set -euo pipefail

WORLD="${1:-smartnav_world}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONUNBUFFERED=1
exec python3 "${SCRIPT_DIR}/reset_cars_loop.py" "$WORLD"
