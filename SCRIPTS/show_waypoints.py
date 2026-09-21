#!/usr/bin/env python3
"""Display the saved points in Gazebo and RViz when a warehouse starts."""

import argparse
import os
import subprocess
import time

import rclpy

from gazebo_waypoint_markers import GazeboWaypointPublisher
from waypoint_file import load_waypoints
from waypoint_markers import WaypointMarkerPublisher


def gazebo_is_ready():
    """Return true after the Gazebo world has exposed its create service."""
    try:
        result = subprocess.run(
            ['gz', 'service', '-l'], text=True, capture_output=True,
            timeout=2, check=True, env=os.environ.copy(),
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return '/world/' in result.stdout and '/create' in result.stdout


def main(points_file):
    try:
        waypoints = load_waypoints(points_file)
    except (FileNotFoundError, ValueError) as error:
        print(f'Метки точек не загружены: {error}', flush=True)
        return 1

    for _ in range(30):
        if gazebo_is_ready():
            break
        time.sleep(1)
    else:
        print('Метки точек не загружены: Gazebo не стал доступен за 30 секунд.', flush=True)
        return 1

    rclpy.init()
    node = rclpy.create_node('warehouse_waypoint_markers')
    try:
        rviz_markers = WaypointMarkerPublisher(node)
        gazebo_markers = GazeboWaypointPublisher(node)
        rviz_markers.publish(waypoints)
        gazebo_markers.publish(waypoints, None)
        node.get_logger().info(f'Отображается точек: {len(waypoints)} ({points_file})')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Show named points in the running warehouse.')
    parser.add_argument('points_file', help='Path to points.txt')
    arguments = parser.parse_args()
    raise SystemExit(main(arguments.points_file))
