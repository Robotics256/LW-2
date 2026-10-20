#!/usr/bin/env python3
"""Launch TurtleBot3 in one of the Gazebo Classic warehouse worlds."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')


def launch_setup(context):
    package_share = get_package_share_directory('turtlebot3_gazebo')
    launch_dir = os.path.join(package_share, 'launch')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world_name = LaunchConfiguration('map').perform(context)
    if not world_name:
        world_name = LaunchConfiguration('world').perform(context)
    worlds = {
        'large': ('turtlebot3_warehouse.world', '-4.3', '-4.5'),
        'small': ('turtlebot3_warehouse_small.world', '-1.75', '-4.5'),
        'stage4': ('turtlebot3_dqn_stage4.world', '0.0', '-2.0'),
        'automotive': ('turtlebot3_warehouse_automotive.world', '0.0', '-20.0'),
    }
    if world_name not in worlds:
        choices = ', '.join(worlds)
        raise RuntimeError(f"Unknown world '{world_name}'. Available: {choices}")

    world_file, default_x, default_y = worlds[world_name]
    x_pose = LaunchConfiguration('x_pose').perform(context) or default_x
    y_pose = LaunchConfiguration('y_pose').perform(context) or default_y
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping', 'navigation'):
        raise RuntimeError("mode must be mapping or navigation")
    lidar_visual = LaunchConfiguration('lidar_visual').perform(context) or str(mode == 'mapping').lower()
    show_waypoints = LaunchConfiguration('show_waypoints').perform(context) or str(mode == 'navigation').lower()
    if lidar_visual.lower() not in ('true', 'false') or show_waypoints.lower() not in ('true', 'false'):
        raise RuntimeError('lidar_visual and show_waypoints must be true or false')
    world_path = os.path.join(package_share, 'worlds', world_file)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': world_path,
                'server_required': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'spawn_turtlebot3.launch.py')),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'lidar_visual': lidar_visual,
            }.items(),
        ),
    ]

    points_file = LaunchConfiguration('points_file').perform(context)
    project_dir = os.environ.get('TB3_PROJECT_DIR')
    marker_script = (
        os.path.join(project_dir, 'SCRIPTS', 'show_waypoints.py')
        if project_dir else '')
    points_file = os.path.expanduser(points_file)
    if (show_waypoints.lower() == 'true' and points_file and marker_script
            and os.path.isfile(points_file) and os.path.isfile(marker_script)):
        actions.append(ExecuteProcess(
            cmd=['python3', marker_script, points_file],
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='small',
            description='Warehouse layout: small, large, stage4, automotive'),
        DeclareLaunchArgument(
            'mode', default_value='navigation',
            description='mapping: visible lidar and hidden waypoint marks; navigation: inverse'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Backward-compatible alias for the world argument'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo Classic client'),
        DeclareLaunchArgument(
            'x_pose', default_value='',
            description='Robot X; empty selects the chosen warehouse parking pose'),
        DeclareLaunchArgument(
            'y_pose', default_value='',
            description='Robot Y; empty selects the chosen warehouse parking pose'),
        DeclareLaunchArgument(
            'points_file', default_value=os.environ.get('TB3_POINTS_FILE', ''),
            description='Optional saved-waypoint file'),
        DeclareLaunchArgument(
            'lidar_visual', default_value='',
            description='Override lidar visibility; empty selects the mode default'),
        DeclareLaunchArgument(
            'show_waypoints', default_value='',
            description='Override saved-point visibility; empty selects the mode default'),
        OpaqueFunction(function=launch_setup),
    ])
