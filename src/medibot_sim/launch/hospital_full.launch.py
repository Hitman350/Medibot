#!/usr/bin/env python3
"""
hospital_full.launch.py — Complete hospital simulation.

Launches Gazebo, robot, bridge, and ALL Nav2 nodes individually
with a properly configured lifecycle manager. No bringup_launch.py —
full control over every node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable,
    AppendEnvironmentVariable, IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Directories ---
    hospital_dir = get_package_share_directory('medibot_sim')
    tb3_dir = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_dir = get_package_share_directory('ros_gz_sim')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # --- Paths ---
    world_file = os.path.join(hospital_dir, 'worlds', 'hospital_ward.world')
    map_yaml = os.path.join(hospital_dir, 'maps', 'hospital_ward.yaml')
    params_file = os.path.join(nav2_dir, 'params', 'nav2_params.yaml')
    urdf_file = os.path.join(tb3_dir, 'urdf', 'turtlebot3_burger.urdf')
    bridge_config = os.path.join(tb3_dir, 'params', 'turtlebot3_burger_bridge.yaml')
    robot_sdf = os.path.join(tb3_dir, 'models', 'turtlebot3_burger', 'model.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # ===================== GAZEBO =====================
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v1 {world_file}',
            'on_exit_shutdown': 'true'
        }.items())

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-g -v1',
            'on_exit_shutdown': 'true'
        }.items())

    # ===================== ROBOT =====================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output='screen')

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'burger', '-file', robot_sdf,
                   '-x', '1.75', '-y', '2.5', '-z', '0.01'],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen')

    # ===================== NAV2 NODES =====================
    # Common params for all Nav2 nodes
    configured_params = params_file
    sim_time = {'use_sim_time': True}

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[configured_params, sim_time,
                    {'yaml_filename': map_yaml}])

    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=[configured_params, sim_time])

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[configured_params, sim_time])

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[configured_params, sim_time])

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[configured_params, sim_time])

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[configured_params, sim_time])

    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server',
        name='smoother_server', output='screen',
        parameters=[configured_params, sim_time])

    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[configured_params, sim_time])

    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[configured_params, sim_time])

    # ===================== LIFECYCLE MANAGER =====================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_nav2',
        output='screen',
        parameters=[{
            'autostart': True,
            'use_sim_time': True,
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'velocity_smoother',
                'collision_monitor',
                'bt_navigator',
            ],
        }])

    # ===================== RVIZ =====================
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'use_sim_time': 'true'}.items())

    # ===================== BUILD =====================
    ld = LaunchDescription()

    # Environment
    ld.add_action(SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'))
    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(tb3_dir, 'models')))

    # Declarations
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))

    # Gazebo
    ld.add_action(gz_server)
    ld.add_action(gz_client)

    # Robot
    ld.add_action(robot_state_pub)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)

    # Nav2 — order matters for lifecycle manager
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(smoother_server)
    ld.add_action(behavior_server)
    ld.add_action(velocity_smoother)
    ld.add_action(collision_monitor)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager)

    # RViz
    ld.add_action(rviz_cmd)

    return ld

