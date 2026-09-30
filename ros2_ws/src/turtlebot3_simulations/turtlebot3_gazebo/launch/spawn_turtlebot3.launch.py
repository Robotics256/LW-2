# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value, argument_name):
    normalized = value.strip().lower()
    if normalized in ('true', '1', 'yes', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f"{argument_name} must be true or false, got: {value}")


def _model_path(model):
    return os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_' + model,
        'model.sdf'
    )


def _model_with_lidar_visibility(model_path, visible):
    """Return the original SDF or a temporary copy with ray visuals hidden."""
    if visible:
        return model_path, None

    tree = ET.parse(model_path)
    ray_sensors = [
        sensor for sensor in tree.findall('.//sensor')
        if sensor.get('type') in ('ray', 'gpu_ray')
    ]
    if not ray_sensors:
        raise RuntimeError(f'No ray lidar sensor found in {model_path}')
    for sensor in ray_sensors:
        visualize = sensor.find('visualize')
        if visualize is None:
            visualize = ET.SubElement(sensor, 'visualize')
        visualize.text = 'false'

    descriptor, temporary_path = tempfile.mkstemp(
        prefix=f'turtlebot3_{os.getpid()}_', suffix='_lidar_hidden.sdf')
    with os.fdopen(descriptor, 'wb') as temporary_file:
        # spawn_entity.py reads -file as a Unicode string. lxml rejects a
        # Unicode string that still contains an XML encoding declaration.
        tree.write(temporary_file, encoding='utf-8', xml_declaration=False)
    return temporary_path, temporary_path


def _remove_temporary_model(_context, temporary_path):
    try:
        os.unlink(temporary_path)
    except FileNotFoundError:
        pass
    return []


def launch_setup(context):
    model = os.environ['TURTLEBOT3_MODEL']
    lidar_visible = _as_bool(
        LaunchConfiguration('lidar_visual').perform(context), 'lidar_visual')
    spawn_model_path, temporary_path = _model_with_lidar_visibility(
        _model_path(model), lidar_visible)

    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model,
            '-file', spawn_model_path,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.01'
        ],
        output='screen',
    )

    actions = [spawner]
    if temporary_path is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=spawner,
            on_exit=[OpaqueFunction(
                function=_remove_temporary_model,
                args=[temporary_path],
            )],
        )))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Initial robot X coordinate'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Initial robot Y coordinate'),
        DeclareLaunchArgument(
            'lidar_visual', default_value='true',
            description='Show Gazebo lidar rays without changing the /scan sensor'),
        OpaqueFunction(function=launch_setup),
    ])
