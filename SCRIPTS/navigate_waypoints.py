#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import os
import sys

def load_waypoints(file_path):
    waypoints = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 4:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(parts[0])
                pose.pose.position.y = float(parts[1])
                pose.pose.orientation.z = float(parts[2])
                pose.pose.orientation.w = float(parts[3])
                waypoints.append(pose)
    return waypoints


def main(target_index=None):
    rclpy.init()
    navigator = BasicNavigator()

    waypoints = load_waypoints(os.path.expanduser('./waypoints.txt'))

    print(f"Загружено {len(waypoints)} точек")
        # Проверка корректности индекса
    if target_index is not None:
        if target_index < 0 or target_index >= len(waypoints):
            print("Неверный номер точки")
            rclpy.shutdown()
            return
        
    navigator.waitUntilNav2Active()
    # Определяем список целей
    if target_index is not None:
        targets = [waypoints[target_index]]
    else:
        targets = waypoints

    for i, wp in enumerate(targets):
        print(f"\nДвижение к точке {i+1}: x={wp.pose.position.x}, y={wp.pose.position.y}")
        navigator.goToPose(wp)
        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Достигнута цель")
        else:
            print(f"Навигация завершилась с результатом: {result}")

    print("Все точки достигнуты")
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    # ===== Чтение аргумента командной строки =====
    target_index = None
    if len(sys.argv) > 1:
        try:
            # пользователь передаёт номер точки с 1, а не с 0
            target_index = int(sys.argv[1]) - 1
        except ValueError:
            print("Аргумент должен быть числом")
    main(target_index)
