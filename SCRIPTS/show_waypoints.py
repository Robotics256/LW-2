#!/usr/bin/env python3
"""Display the saved points in Gazebo and RViz when a warehouse starts."""

import argparse

import rclpy

from gazebo_waypoint_markers import GazeboWaypointPublisher
from waypoint_file import load_waypoints
from waypoint_markers import WaypointMarkerPublisher


def main(points_file):
    try:
        waypoints = load_waypoints(points_file)
    except (FileNotFoundError, ValueError) as error:
        print(f'Метки точек не загружены: {error}', flush=True)
        return 1

    rclpy.init()
    node = rclpy.create_node('warehouse_waypoint_markers')
    try:
        rviz_markers = WaypointMarkerPublisher(node)
        gazebo_markers = GazeboWaypointPublisher(node)
        if not gazebo_markers.wait_until_ready(timeout_sec=30.0):
            node.get_logger().error(
                'Метки точек не загружены: Gazebo не стал доступен за 30 секунд.')
            return 1
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
