#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_description = FindPackageShare('smartnav_description')
    pkg_gazebo = FindPackageShare('smartnav_gazebo')
    
    urdf_xacro = PathJoinSubstitution([pkg_description, 'urdf', 'smartnav.xacro'])
    world_file = PathJoinSubstitution([pkg_gazebo, 'worlds', 'smartnav.world'])
    rviz_config_file = os.path.join(
        get_package_share_directory('smartnav_description'),
        'rviz',
        'smartnav.rviz'
    )
    robot_description = Command(['xacro ', urdf_xacro])
    
    # 1. Lancer Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file],
        output='screen'
    )
    
    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # 3. Spawn le robot après 8 secondes (TimerAction fonctionne toujours)
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
    
    # 4. Bridge LiDAR
    ros_gz_bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )
    
    # 5. Bridge cmd_vel
    ros_gz_bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    
    # 6. Bridge horloge
    ros_gz_bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )
    
    # 7. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge_lidar,
        ros_gz_bridge_cmd_vel,
        ros_gz_bridge_clock,
        rviz2
    ])
