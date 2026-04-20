# SmartNav: Smart White Cane Simulation

## Project Overview
The SmartNav project is a robotic simulation platform designed to model and test a smart white cane for visually impaired individuals. Developed using ROS 2 and Gazebo, the system integrates the physical simulation of a user equipped with an instrumented cane (LiDAR sensor) and real-time perception algorithms for obstacle detection and alerting.

## System Architecture
The workspace is structured around three main ROS 2 packages:
- smartnav_description: Kinematic and visual definition of the system (URDF/XACRO files).
- smartnav_gazebo: 3D simulation environment, physics management, and obstacle instantiation.
- smartnav_core: Algorithmic nodes (Python) dedicated to sensor data processing and control logic.

## Technical Stack and Tools
- Robotics Framework: ROS 2 (Jazzy Jalisco)
- 3D Simulator: Gazebo (Harmonic)
- Visualization Tool: RViz2
- Programming Languages: Python 3 (Algorithms), XML/XACRO (Modeling)
- Build System: colcon
- Version Control: Git

## Prerequisites and Dependencies
- Operating System: Ubuntu 24.04 LTS
- ROS 2 Jazzy (Desktop installation recommended)
- Standard ROS 2 tools: `python3-colcon-common-extensions`, `python3-rosdep`

## Installation

1. Initialize the workspace and clone the repository:
```bash
mkdir -p ~/smartnav_ws/src
cd ~/smartnav_ws/src
git clone <YOUR_GIT_REPOSITORY_URL> .
Update and install system dependencies via rosdep:

Bash
cd ~/smartnav_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
Build the project:

Bash
cd ~/smartnav_ws
colcon build --symlink-install
Source the environment:

Bash
source install/setup.bash
Usage and Execution
Running the full system requires launching the simulation environment and the processing nodes in parallel. Open separate terminals and ensure you run source install/setup.bash in each of them before executing the commands.

Terminal 1: Launch the physical simulation (Gazebo) and visualization interface (RViz2)

Bash
ros2 launch smartnav_core start_simulation.launch.py
Terminal 2: Activate the perception module (LiDAR data analysis)

Bash
ros2 run smartnav_core obstacle_detector
Terminal 3: Activate the mobility scenarios (Movement and testing)

Bash
ros2 run smartnav_core patrol
Repository Structure
The source code follows the standard architecture of a ROS 2 workspace:

Plaintext
src/
├── smartnav_core/          # Perception and mobility nodes (Python)
├── smartnav_description/   # 3D Models (URDF/XACRO) and RViz configurations
├── smartnav_gazebo/        # World files (.world) and Gazebo launch files
└── README.md               # Main documentation
