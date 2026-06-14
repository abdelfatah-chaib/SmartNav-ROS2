# SmartNav: Smart Navigational Cane Simulation

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-34a853?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-ff6600)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?logo=ubuntu)

## Project Overview

The **SmartNav** project is an advanced robotic simulation platform designed to model and test a smart navigational cane for visually impaired individuals. Developed using ROS 2 and Gazebo, the system integrates the physical simulation of a user equipped with an instrumented cane (LiDAR sensor) and real-time perception algorithms for autonomous obstacle detection, alerting, and mobility testing.

## Key Features

- **Realistic 3D Simulation:** Immersive urban environment (`city.world`) featuring dynamic elements like moving cars and pedestrians.
- **LiDAR Perception:** 360° simulated sensor for real-time distance calculation and obstacle tracking.
- **Tiered Alert System:** Multi-level danger zones based on proximity (Clear, Warning, Critical).
- **Automated Mobility Testing:** Built-in patrol nodes to test sensor reactions against environmental obstacles.
- **Visual Feedback:** Full RViz2 integration for real-time sensor data debugging and system state visualization.

## System Architecture

The workspace is structured around three main ROS 2 packages:

-  `smartnav_description`: Kinematic and visual definition of the system (URDF/XACRO files) and RViz configurations.
-  `smartnav_gazebo`: 3D simulation environment, physics management, SDF world files, and obstacle instantiation.
-  `smartnav_core`: The "Brain" of the project. Algorithmic nodes (Python) dedicated to sensor data processing, control logic, and telemetry.

###  ROS 2 Interface (Topics)

| Topic | Type | Publisher | Subscriber | Description |
|---------|---------|---------|---------|---------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo (LiDAR) | `obstacle_detector` | Raw distance measurements. |
| `/smartnav/alert_level` | `std_msgs/Int8` | `obstacle_detector` | (Future UI/RViz nodes) | Alert level: 0 (Safe), 1 (Warning), 2 (Danger). |
| `/cmd_vel` | `geometry_msgs/Twist` | `patrol` / `teleop` | Gazebo (Base) | Velocity commands to move the simulated user. |

##  Technical Stack and Tools

- **Robotics Framework:** ROS 2 (Jazzy Jalisco)
- **3D Simulator:** Gazebo (Harmonic)
- **Visualization Tool:** RViz2
- **Programming Languages:** Python 3 (Algorithms), XML/XACRO (Modeling)
- **Build System:** `colcon`

##  Prerequisites and Dependencies

- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy (Desktop installation highly recommended)
- **Standard Tools:** `python3-colcon-common-extensions`, `python3-rosdep`

##  Installation

### 1. Initialize the workspace and clone the repository

```bash
mkdir -p ~/smartnav_ws/src
cd ~/smartnav_ws/src
git clone https://github.com/abdelfatah-chaib/SmartNav-ROS2.git .
```

### 2. Update and install system dependencies via rosdep

```bash
cd ~/smartnav_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the project

```bash
cd ~/smartnav_ws
colcon build --symlink-install
```

### 4. Source the environment

```bash
source install/setup.bash
```

##  Usage and Execution

Running the full system requires launching the simulation environment and the processing nodes in parallel.

>  **Note:** Open separate terminals and run `source ~/smartnav_ws/install/setup.bash` in each terminal before executing the commands below.

### Terminal 1: Launch the physical simulation (Gazebo) and visualization interface (RViz2)

```bash
ros2 launch smartnav_core start_simulation.launch.py
```

*(By default, this launches `city.world`. To force the legacy scene, append `world:=smartnav.world` to the command).*

### Terminal 2: Activate the perception module (LiDAR data analysis)

```bash
ros2 run smartnav_core obstacle_detector
```

### Terminal 3: Activate mobility scenarios (Movement and testing)

```bash
ros2 run smartnav_core patrol
```

##  Repository Structure

```text
SmartNav-ROS2/
├── smartnav_core/          # Perception, FSM, and mobility nodes (Python)
├── smartnav_description/   # 3D models (URDF/XACRO) and RViz configurations
├── smartnav_gazebo/        # World files (.world) and Gazebo launch scripts
├── README.md               # Main documentation
└── ROBOT_MOOUVEMENT.md     # Specific documentation for movement and debugging
```

##  Troubleshooting

### No objects in Gazebo / Black screen

Ensure you have an active internet connection the first time you launch the `city.world`, as Gazebo needs to download the "Sun" and some environmental models from Gazebo Fuel.

### Colcon build warnings/errors

Make sure you do not have nested `build/` or `install/` folders inside your `src/` directory. If you do, delete them:

```bash
rm -rf src/build src/install
```

Then run:

```bash
colcon build
```

again from the root of the workspace.

### Robot not moving

Verify that your terminal is publishing to `/cmd_vel` without a leading namespace issue, and that the simulation is not paused in Gazebo.

##  Contributing

Contributions, issues, and feature requests are welcome!

Feel free to check the issues page.

## 📄 License

This project is licensed under the MIT License - see the `LICENSE` file for details.
