#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = FindPackageShare('smartnav_description')
    pkg_gazebo = FindPackageShare('smartnav_gazebo')
    world = LaunchConfiguration('world')

    urdf_xacro = PathJoinSubstitution([pkg_description, 'urdf', 'smartnav.xacro'])
    world_file = PathJoinSubstitution([pkg_gazebo, 'worlds', world])
    # Gazebo publishes LiDAR on this fully-qualified topic for the spawned robot.
    # We bridge it and remap it back to /scan for ROS nodes.
    gz_lidar_topic = (
        'world/smartnav_world/model/smartnav/link/base_link/sensor/laser_sensor/scan'
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('smartnav_description'),
        'rviz',
        'smartnav.rviz'
    )
    robot_description = Command(['xacro ', urdf_xacro])
    reset_cars_script = os.path.join(
        get_package_share_directory('smartnav_gazebo'),
        'worlds',
        'reset_cars.sh'
    )
    walk_blind_person_script = os.path.join(
        get_package_share_directory('smartnav_gazebo'),
        'worlds',
        'walk_blind_person.sh'
    )

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

    # 3. Le robot smartnav est maintenant défini directement dans city.world (SDF).
    #    Cela garantit que VelocityControl est initialisé avec un topic absolu
    #    (/smartnav/cmd_vel) accessible depuis tout processus externe.
    #    Plus besoin de spawn dynamique (qui causait des échecs de découverte gz-transport).

    # 3b. Bouclage position-based des voitures (reset_cars_loop.py).
    #     Chaque voiture parcourt toute la route puis est replacée au départ.
    #     Lancé 5 s après Gazebo pour laisser le moteur physique s'initialiser.
    reset_cars = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', reset_cars_script],
                output='log'
            )
        ]
    )

    # 3c. Contrôleur de trajectoire du piéton (traverse puis tourne sur trottoir)
    walk_blind_person = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', walk_blind_person_script],
                output='log'
            )
        ]
    )

    # 4. Bridge LiDAR — robot défini dans le world SDF → topic disponible dès le démarrage.
    #    5 s suffisent pour que Gazebo charge le world et initialise le capteur.
    ros_gz_bridge_lidar = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'{gz_lidar_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                ],
                remappings=[(gz_lidar_topic, '/scan')],
                output='screen',
            )
        ],

    )

ros_gz_bridge_poses = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/model/smartnav/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
                    '/model/moving_car_A/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
                    '/model/moving_car_B/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
                ],
                output='screen',
            )
        ],
    )

    # 5. Relay cmd_vel — robot dans le world SDF, VelocityControl prêt après 5 s.
    cmd_vel_relay = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='smartnav_core',
                executable='cmd_vel_relay',
                name='cmd_vel_relay',
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    # 6. Bridge horloge — GZ→ROS uniquement ([) pour éviter la boucle de republication
    #    qui génère un flood de "Host unreachable" dans Gazebo.
    ros_gz_bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 6b. Transform statique manquant: base_link -> laser frame Gazebo scoped
    #     Corrige les "Message Filter dropping" dans RViz.
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'smartnav/base_link/laser_sensor'],
        output='screen',
    )

    # 7. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 8. Obstacle detector: LiDAR → alert level (Int8)
    obstacle_detector = Node(
        package='smartnav_core',
        executable='obstacle_detector',
        name='obstacle_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 9. RViz alert visualizer: alert level → cane color marker + vibration visual
    rviz_alert_node = Node(
        package='smartnav_core',
        executable='rviz_alert_node',
        name='rviz_alert_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 9b. Feedback canne: alert level → son audio + topic vibration
    canne_feedback = Node(
        package='smartnav_core',
        executable='canne_feedback',
        name='canne_feedback_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 9c. Crossing monitor: alerte voiture au passage pieton -> Int8 danger/clear
    crossing_monitor = Node(
        package='smartnav_core',
        executable='crossing_monitor',
        name='crossing_monitor_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 10. Autonomous navigator: reactive FSM (WANDER / AVOIDANCE)
    #     use_sim_time=True pour cohérence horloge Gazebo ↔ watchdog timer.
    smartnav_navigator = Node(
        package='smartnav_core',
        executable='smartnav_navigator',
        name='smartnav_navigator',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='city.world',
            description='Gazebo world file from smartnav_gazebo/worlds',
        ),
        start_gazebo,
        robot_state_publisher,
        reset_cars,
        walk_blind_person,
        ros_gz_bridge_lidar,
        cmd_vel_relay,
        ros_gz_bridge_clock,
        static_tf_laser,
        rviz2,
        obstacle_detector,
        rviz_alert_node,
        canne_feedback,
        crossing_monitor,
        smartnav_navigator,
    ])
