#!/bin/bash
# ============================================================
#  reset_cars.sh — Bouclage des voitures dynamiques SmartNav
#  Usage : bash reset_cars.sh
#  Lancer dans un terminal séparé APRÈS gz sim smartnav_world_v6.world
# ============================================================

WORLD="smartnav_world"
PERIOD=16         # reset toutes les 16 s (road~120m / 7m·s⁻¹ = 17.1s)

# Quaternion yaw=0      → w=1 z=0  (moving_car_A, W→E)
# Quaternion yaw=π      → w=0 z=1  (moving_car_B, E→W)

echo "╔══════════════════════════════════════════════════════╗"
echo "║   SmartNav — Reset automatique des voitures          ║"
echo "║   Monde  : $WORLD                              ║"
echo "║   Période: ${PERIOD}s  │  Ctrl+C pour arrêter          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# Attente initiale pour laisser Gazebo démarrer
echo "⏳ Attente 3 s pour que Gazebo soit prêt..."
sleep 3

while true; do
  sleep $PERIOD

  # ── Voiture A (ROUGE) : retour à l'ouest x=-58, voie nord y=+2 ──
  gz service -s /world/$WORLD/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --req "name: 'moving_car_A'
           position { x: -58.0  y: 2.0  z: 0.54 }
           orientation { x: 0.0  y: 0.0  z: 0.0  w: 1.0 }" \
    --timeout 1000 > /dev/null 2>&1

  echo "$(date +%H:%M:%S)  ↺  moving_car_A  → x=-58  (W→E)"

  # ── Voiture B (BLEUE) : retour à l'est x=+58, voie sud y=-2 ──
  gz service -s /world/$WORLD/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --req "name: 'moving_car_B'
           position { x: 58.0  y: -2.0  z: 0.54 }
           orientation { x: 0.0  y: 0.0  z: 1.0  w: 0.0 }" \
    --timeout 1000 > /dev/null 2>&1

  echo "$(date +%H:%M:%S)  ↺  moving_car_B  → x=+58  (E→W)"
done
