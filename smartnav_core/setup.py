from setuptools import find_packages, setup

package_name = 'smartnav_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdelfatah',
    maintainer_email='abdelfatahchaib@gmail.com',
    description=(
        'SmartNav core package: obstacle detection, alert visualization, '
        'and autonomous navigation for a smart white cane simulation.'
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
        ],
    },
)
