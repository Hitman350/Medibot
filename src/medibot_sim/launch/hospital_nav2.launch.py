#!/usr/bin/env python3
"""
hospital_nav2.launch.py — Hospital simulation using Nav2's proven launch.

Uses nav2_bringup's tb3_simulation_launch.py (which handles lifecycle
management correctly) but overrides the world and map to our hospital.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    medibot_sim_dir = get_package_share_directory('medibot_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Use Nav2's proven TB3 simulation launch with our hospital overrides
    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch',
                         'tb3_simulation_launch.py')),
        launch_arguments={
            'world': os.path.join(medibot_sim_dir, 'worlds',
                                  'hospital_ward.world'),
            'map': os.path.join(medibot_sim_dir, 'maps',
                                'hospital_ward.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_rviz': 'true',
            'x_pose': '1.75',
            'y_pose': '2.5',
            'z_pose': '0.01',
            'yaw': '0.0',
        }.items())

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        nav2_sim,
    ])

