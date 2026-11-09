#!/usr/bin/env python3
"""
hospital_sim.launch.py -- All-in-one hospital AMR launch.

Launches Gazebo (hospital ward), TurtleBot3 Burger, Nav2, and RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    AppendEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    medibot_sim_dir = get_package_share_directory('medibot_sim')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(medibot_sim_dir, 'worlds', 'hospital_ward.world')
    map_yaml = os.path.join(medibot_sim_dir, 'maps', 'hospital_ward.yaml')
    nav2_params = os.path.join(medibot_sim_dir, 'config', 'nav2_hospital_params.yaml')

    # Use TurtleBot3 Burger
    urdf_file = os.path.join(tb3_gazebo_dir, 'urdf', 'turtlebot3_burger.urdf')
    bridge_config = os.path.join(tb3_gazebo_dir, 'params', 'turtlebot3_burger_bridge.yaml')
    robot_sdf = os.path.join(tb3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_rviz = LaunchConfiguration('use_rviz', default='true')

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    append_gz_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models'))

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world_file}',
            'on_exit_shutdown': 'true'
        }.items())

    # Gazebo GUI
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true'
        }.items())

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }])

    # Spawn robot at pharmacy (1.75, 2.5) — original working position
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'burger',
            '-file', robot_sdf,
            '-x', '1.75',
            '-y', '2.5',
            '-z', '0.05',
        ],
        output='screen')

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
        output='screen')

    # Nav2 bringup (composition mode -- proven working)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml,
            'params_file': nav2_params,
            'autostart': 'true',
            'use_composition': 'true',
            'use_respawn': 'false',
            'use_keepout_zones': 'false',
            'use_speed_zones': 'false',
            'namespace': '',
        }.items())

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': '',
        }.items())

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(set_tb3_model)
    ld.add_action(append_gz_path)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz_launch)
    return ld
