#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.msg import SpeedLimit
from rclpy.parameter import Parameter
import argparse
import time

from waypoint_file import waypoints_by_name
from gazebo_trail import GazeboTrail
from lw2_config import DEFAULT_CONFIG_FILE
from navigation_modes import add_mode_arguments, add_trail_arguments, behavior_tree, validate_trail


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


def wait_for_navigation_without_resetting_pose(navigator):
    """Wait for Humble Nav2 without publishing BasicNavigator's (0, 0) pose."""
    navigator._waitForNodeToActivate('amcl')
    navigator._waitForNodeToActivate('bt_navigator')
    if not navigator.initial_pose_received:
        print(
            "Ожидание начальной позы AMCL. В RViz выберите "
            "2D Pose Estimate и укажите P2."
        )
        while not navigator.initial_pose_received:
            rclpy.spin_once(navigator, timeout_sec=0.2)


def main(
    points_file,
    point_names,
    speed,
    navigation_mode,
    trail_color_number,
    config_file,
):
    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter("use_sim_time", value=True)])
    trail = (
        GazeboTrail(
            navigator,
            color_number=trail_color_number,
            config_file=config_file,
        )
        if trail_color_number is not False else None
    )

    try:
        waypoints = waypoints_by_name(points_file)
        missing = [name for name in point_names if name not in waypoints]
        if missing:
            available = ", ".join(waypoints) or "нет"
            raise ValueError(f"Неизвестные точки: {', '.join(missing)}. Доступны: {available}")

        print(f"Загружено {len(waypoints)} точек из {points_file}")
        wait_for_navigation_without_resetting_pose(navigator)
        speed_limit_publisher = publish_speed_limit(navigator, speed)
        print(f"Ограничение линейной скорости: {speed:.2f} м/с")
        print(f"Режим прибытия: {navigation_mode}")
        if trail is not None:
            print("Пунктирный след в Gazebo включён")
        for name in point_names:
            waypoint = waypoints[name]
            pose = waypoint["pose"]
            pose.header.stamp = navigator.get_clock().now().to_msg()
            print(f"\nДвижение к '{name}': x={pose.pose.position.x}, y={pose.pose.position.y}")
            if trail is not None:
                trail.start(name)
            accepted = navigator.goToPose(
                pose,
                behavior_tree=behavior_tree(navigation_mode),
            )
            if not accepted:
                raise RuntimeError(f"Nav2 отклонил цель '{name}'")
            while not navigator.isTaskComplete():
                if trail is not None:
                    trail.sample()
                rclpy.spin_once(navigator)

            if trail is not None:
                trail.stop()

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
    add_mode_arguments(parser)
    add_trail_arguments(parser)
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_FILE),
        help="Общий JSON-конфиг LW-2",
    )
    arguments = parser.parse_args()
    if not 0.0 < arguments.speed <= 0.30:
        parser.error("--speed должна быть больше 0 и не больше 0.30 м/с")
    validate_trail(parser, arguments.trail)
    main(
        arguments.points_file,
        arguments.points,
        arguments.speed,
        arguments.navigation_mode,
        arguments.trail,
        arguments.config,
    )
