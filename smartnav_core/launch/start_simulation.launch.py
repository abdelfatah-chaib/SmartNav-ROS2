#!/usr/bin/env python3
"""
SmartNav - Main Simulation Launch File (v2)
===========================================
Launches:
  1. Gazebo Harmonic with highway_forest.world
  2. Robot State Publisher (humanoid v2 URDF)
  3. Robot spawn in Gazebo
  4. ROS-Gazebo bridges (scan, scan_chest, cmd_vel, clock, car topics)
  5. RViz2 for visualization
  6. Obstacle Detector node
  7. Haptic Feedback simulation node
  8. Car Controller node (manages dynamic highway cars)

Use:
  ros2 launch smartnav_core start_simulation.launch.py
  ros2 launch smartnav_core start_simulation.launch.py world:=smartnav
  ros2 launch smartnav_core start_simulation.launch.py use_v2_urdf:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, TimerAction, DeclareLaunchArgument
)
from launch.substitutions import (
    Command, PathJoinSubstitution, LaunchConfiguration
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ─────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world', default_value='highway_forest',
        description='World name (without .world extension)'
    )
    use_v2_arg = DeclareLaunchArgument(
        'use_v2_urdf', default_value='true',
        description='Use improved humanoid URDF v2 (true) or original (false)'
    )

    pkg_description = FindPackageShare('smartnav_description')
    pkg_gazebo      = FindPackageShare('smartnav_gazebo')

    # ── URDF selection ────────────────────────────────────────────
    urdf_v2   = PathJoinSubstitution([pkg_description, 'urdf', 'smartnav_v2.xacro'])
    urdf_v1   = PathJoinSubstitution([pkg_description, 'urdf', 'smartnav.xacro'])

    # Default to v2
    urdf_xacro  = urdf_v2
    world_name  = LaunchConfiguration('world')

    rviz_config = os.path.join(
        get_package_share_directory('smartnav_description'),
        'rviz', 'smartnav.rviz'
    )

    robot_description = Command(['xacro ', urdf_xacro])

    # ── 1. Gazebo ─────────────────────────────────────────────────
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r',
             PathJoinSubstitution([pkg_gazebo, 'worlds',
                                   [world_name, '.world']])],
        output='screen'
    )

    # ── 2. Robot State Publisher ──────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True
        }],
        output='screen'
    )

    # ── 3. Spawn robot (after 8s) ─────────────────────────────────
    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'smartnav',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    # ── 4. Bridges ────────────────────────────────────────────────
    # LiDAR cane (ground level)
    bridge_lidar_cane = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # LiDAR chest (body/head level) - new in v2
    bridge_lidar_chest = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan_chest@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # cmd_vel (robot motion control)
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    # Clock (simulation time)
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )

    # Car cmd_vel bridges (for dynamic cars in highway_forest world)
    bridge_car_red = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/car_red_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    bridge_car_blue = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/car_blue_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    bridge_car_white = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/car_white_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    # ── 5. RViz2 ──────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ── 6. Perception nodes (start after spawn) ───────────────────
    obstacle_detector = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='smartnav_core',
                executable='obstacle_detector',
                output='screen',
                parameters=[{
                    'danger_dist_cane':   0.50,
                    'warning_dist_cane':  1.20,
                    'danger_dist_chest':  0.80,
                    'warning_dist_chest': 2.00,
                    'front_half_angle':   60.0,
                    'emergency_stop':     True,
                }]
            )
        ]
    )

    haptic_feedback = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='smartnav_core',
                executable='haptic_feedback',
                output='screen'
            )
        ]
    )

    rviz_alert = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='smartnav_core',
                executable='rviz_alert_node',
                output='screen'
            )
        ]
    )

    # ── 7. Car Controller (starts after Gazebo is ready) ─────────
    car_controller = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='smartnav_core',
                executable='car_controller',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        use_v2_arg,
        start_gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge_lidar_cane,
        bridge_lidar_chest,
        bridge_cmd_vel,
        bridge_clock,
        bridge_car_red,
        bridge_car_blue,
        bridge_car_white,
        rviz2,
        obstacle_detector,
        haptic_feedback,
        rviz_alert,
        car_controller,
    ])
