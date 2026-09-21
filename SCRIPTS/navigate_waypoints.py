#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.msg import SpeedLimit
import argparse
import time

from waypoint_file import waypoints_by_name


def publish_speed_limit(navigator, speed):
    """Set an absolute controller speed limit in metres per second."""
    publisher = navigator.create_publisher(SpeedLimit, "/speed_limit", 10)
    message = SpeedLimit()
    message.header.stamp = navigator.get_clock().now().to_msg()
    message.percentage = False
    message.speed_limit = speed
    # Give discovery a moment and publish more than once so the active
    # controller reliably receives the limit before the first goal.
    for _ in range(3):
        publisher.publish(message)
        rclpy.spin_once(navigator, timeout_sec=0.1)
        time.sleep(0.05)
    return publisher


def main(points_file, point_names, speed):
    rclpy.init()
    navigator = BasicNavigator()

    try:
        waypoints = waypoints_by_name(points_file)
        missing = [name for name in point_names if name not in waypoints]
        if missing:
            available = ", ".join(waypoints) or "нет"
            raise ValueError(f"Неизвестные точки: {', '.join(missing)}. Доступны: {available}")

        print(f"Загружено {len(waypoints)} точек из {points_file}")
        # ``waitUntilNav2Active()`` normally calls AMCL's initial-pose helper.
        # This command-line client must not overwrite the pose selected by the
        # operator with RViz's 2D Pose Estimate, so wait only for bt_navigator.
        navigator.waitUntilNav2Active(localizer="robot_localization")
        # AMCL creates map -> odom only after the operator supplies the initial
        # pose.  Waiting here avoids sending a goal that must fail because the
        # robot has no pose in the map frame yet.
        if not navigator.initial_pose_received:
            print(
                "Ожидание начальной позы AMCL. В RViz выберите "
                "2D Pose Estimate и укажите P2."
            )
            while not navigator.initial_pose_received:
                rclpy.spin_once(navigator, timeout_sec=0.2)
        speed_limit_publisher = publish_speed_limit(navigator, speed)
        print(f"Ограничение линейной скорости: {speed:.2f} м/с")
        for name in point_names:
            waypoint = waypoints[name]
            pose = waypoint["pose"]
            print(f"\nДвижение к '{name}': x={pose.pose.position.x}, y={pose.pose.position.y}")
            navigator.goToPose(pose)
            while not navigator.isTaskComplete():
                rclpy.spin_once(navigator)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"Точка '{name}' достигнута")
            else:
                print(f"Навигация к '{name}' завершилась с результатом: {result}")
                break
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Send named points from points.txt directly to Nav2 (without MQTT)."
    )
    parser.add_argument("points_file", help="Path to points.txt")
    parser.add_argument("points", nargs="+", help="Waypoint names, for example: t1 p1")
    parser.add_argument(
        "--speed", type=float, default=0.20,
        help="Maximum linear speed in m/s (default: 0.20; warehouse maximum: 0.30)",
    )
    arguments = parser.parse_args()
    if not 0.0 < arguments.speed <= 0.30:
        parser.error("--speed должна быть больше 0 и не больше 0.30 м/с")
    main(arguments.points_file, arguments.points, arguments.speed)
