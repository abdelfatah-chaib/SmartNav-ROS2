# SmartNav — Canne Blanche Intelligente ROS 2 / Gazebo

Simulation ROS 2 d'une canne blanche intelligente pour la navigation autonome
d'une personne malvoyante en milieu urbain (passage piéton, voitures, obstacles statiques).

## 📦 Architecture des packages

```
smartnav_ws/src/
├── smartnav_core/          # Logique ROS 2 (nœuds Python)
│   ├── smartnav_core/
│   │   ├── obstacle_detector.py        # Détection LiDAR → alert_level
│   │   ├── smartnav_navigator.py       # FSM 5 états (navigateur autonome)
│   │   ├── crossing_monitor_node.py    # Surveillance voitures Gazebo
│   │   ├── canne_feedback_node.py      # Son + vibration
│   │   ├── rviz_alert_node.py          # Visualisation RViz (marqueurs)
│   │   ├── metrics_recorder_node.py    # Enregistrement CSV des métriques [NOUVEAU]
│   │   ├── battery_simulator_node.py   # Simulateur batterie visuel [NOUVEAU]
│   │   └── cmd_vel_relay.py            # Relay cmd_vel → Gazebo
│   ├── config/
│   │   └── smartnav_params.yaml        # Paramètres centralisés [NOUVEAU]
│   ├── launch/
│   │   └── start_simulation.launch.py  # Launch complet
│   └── test/
│       ├── test_obstacle_detector.py   # Tests unitaires (cas limites ajoutés)
│       └── test_smartnav_navigator.py
├── smartnav_gazebo/        # Monde Gazebo (city.world, scripts voitures)
└── smartnav_description/   # URDF/xacro du robot, config RViz
```

## 🗺️ Topics ROS 2

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `LaserScan` | Données brutes LiDAR |
| `/smartnav/alert_level` | `Int8` | Niveau alerte LiDAR (0=libre, 1=attention, 2=danger) |
| `/smartnav/crossing_alert` | `Int8` | Alerte voiture au passage piéton |
| `/smartnav/state` | `String` | État FSM courant (WANDER/AVOIDANCE/DANGER_HOLD…) |
| `/smartnav/vibration` | `Int8` | Commande vibration canne |
| `/smartnav/battery` | `Float32` | Niveau batterie simulé (%) |
| `/smartnav/feedback_log` | `String` | Log horodaté des alertes |
| `/smartnav/canne_marker` | `Marker` | Marqueur cylindre canne RViz |
| `/smartnav/battery_marker` | `Marker` | Barre batterie RViz |
| `/smartnav/trail_marker` | `Marker` | Trajectoire passée (LINE_STRIP) |
| `/smartnav/zone_markers` | `MarkerArray` | Zones d'alerte concentriques RViz |
| `/cmd_vel` | `Twist` | Commandes vitesse robot |

## 🚀 Lancement

```bash
# Build
cd smartnav_ws
colcon build --symlink-install
source install/setup.bash

# Lancement standard
ros2 launch smartnav_core start_simulation.launch.py

# Sans son (utile en test)
ros2 launch smartnav_core start_simulation.launch.py audio_enabled:=false

# Monde alternatif
ros2 launch smartnav_core start_simulation.launch.py world:=smartnav.world
```

## 🔍 Monitoring en temps réel

```bash
# État FSM du navigateur
ros2 topic echo /smartnav/state

# Niveau d'alerte LiDAR
ros2 topic echo /smartnav/alert_level

# Batterie
ros2 topic echo /smartnav/battery

# Log des alertes
ros2 topic echo /smartnav/feedback_log
```

## 📊 Métriques de session

Chaque session génère automatiquement un fichier CSV dans `/tmp/` :

```bash
# Exemple de chemin
/tmp/smartnav_metrics_20260615_143022.csv

# Colonnes : timestamp, event_type, value, detail, session_danger_count, session_attention_count
```

## ⚙️ Paramètres configurables

Tous les seuils sont dans `config/smartnav_params.yaml` — modifiable sans recompiler :

```yaml
obstacle_detector:
  ros__parameters:
    front_cone_deg: 35.0       # Angle du cône de détection
    danger_threshold_m: 0.5    # Seuil danger (m)
    warn_threshold_m: 1.2      # Seuil attention (m)

crossing_monitor_node:
  ros__parameters:
    danger_radius_m: 3.5       # Rayon danger voiture (m)
    warn_radius_m: 6.0         # Rayon attention voiture (m)

canne_feedback_node:
  ros__parameters:
    audio_enabled: true        # Activer/désactiver le son
```

## 🧪 Tests unitaires

```bash
cd smartnav_ws
colcon test --packages-select smartnav_core
colcon test-result --verbose
```

## 📡 Demo soutenance — Enregistrement rosbag

```bash
# Enregistrer une session complète (rejouable si Gazebo plante)
ros2 bag record -a -o smartnav_demo_session

# Rejouer
ros2 bag play smartnav_demo_session
```

## 🏗️ FSM Navigateur (5 états)

```
                    ┌─────────────────────────────────────────────┐
                    │              WANDER (avance)                │
                    └──────┬──────────────────────────────────────┘
              alert_level=2│          front_min < 0.75m
                           ▼                    ▼
                    ┌─────────────┐    ┌──────────────────┐
                    │ DANGER_HOLD │    │ OBSTACLE_DETECTED │
                    │  (arrêt)   │    └────────┬─────────┘
                    └──────┬──────┘             │ immédiat
          N cycles level=0 │             ┌──────▼────────────┐
                           │             │  ROTATE_AND_SCAN  │ ← NOUVEAU
                           │             │  (mesure L/R)     │
                           │             └──────┬────────────┘
                           │          direction choisie
                           │                   ▼
                           │          ┌─────────────────┐
                           │          │    AVOIDANCE    │
                           │          │   (contourne)   │
                           │          └──────┬──────────┘
                           │  voie libre N cycles
                           └──────────────────┘
```
