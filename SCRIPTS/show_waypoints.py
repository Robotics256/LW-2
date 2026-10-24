#!/usr/bin/env python3
"""Display the saved points in Gazebo and RViz when a warehouse starts."""

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter

from gazebo_waypoint_markers import GazeboWaypointPublisher
from waypoint_file import load_waypoints
from waypoint_markers import WaypointMarkerPublisher


def main(points_file, rviz_only=False):
    try:
        waypoints = load_waypoints(points_file)
    except (FileNotFoundError, ValueError) as error:
        print(f'Метки точек не загружены: {error}', flush=True)
        return 1

    rclpy.init()
    node = rclpy.create_node(
        'navigation_waypoint_markers' if rviz_only else 'warehouse_waypoint_markers',
        parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        rviz_markers = WaypointMarkerPublisher(node)
        rviz_markers.publish(waypoints)
        # Also support RViz displays saved with volatile durability.
        node.create_timer(1.0, lambda: rviz_markers.publish(waypoints))
        if not rviz_only:
            gazebo_markers = GazeboWaypointPublisher(node)
            if gazebo_markers.wait_until_ready(timeout_sec=30.0):
                gazebo_markers.publish(waypoints, None)
            else:
                node.get_logger().warning(
                    'Gazebo недоступен; точки отображаются только в RViz.')
        node.get_logger().info(f'Отображается точек: {len(waypoints)} ({points_file})')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Show named points in the running warehouse.')
    parser.add_argument('points_file', help='Path to points.txt')
    parser.add_argument('--rviz-only', action='store_true', help='Do not wait for Gazebo')
    arguments = parser.parse_args()
    raise SystemExit(main(arguments.points_file, arguments.rviz_only))
