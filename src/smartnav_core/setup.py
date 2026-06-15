from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smartnav_core'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_simulation.launch.py']),
        # Fichier de parametres centralise
        ('share/' + package_name + '/config', ['config/smartnav_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdelfatah',
    maintainer_email='abdelfatahchaib@gmail.com',
    description=(
        'SmartNav core package v2: obstacle detection, alert visualization, '
        'autonomous navigation, metrics recording and battery simulation '
        'for a smart white cane ROS2/Gazebo simulation.'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detector = smartnav_core.obstacle_detector:main',
            'rviz_alert_node = smartnav_core.rviz_alert_node:main',
            'canne_feedback = smartnav_core.canne_feedback_node:main',
            'crossing_monitor = smartnav_core.crossing_monitor_node:main',
            'cmd_vel_relay = smartnav_core.cmd_vel_relay:main',
            'patrol = smartnav_core.patrol:main',
            'smartnav_navigator = smartnav_core.smartnav_navigator:main',
            # Nouveaux noeuds v2
            'metrics_recorder = smartnav_core.metrics_recorder_node:main',
            'battery_simulator = smartnav_core.battery_simulator_node:main',
        ],
    },
)
