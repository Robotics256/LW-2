#!/usr/bin/env python3
"""Удалить существующие точки следа движения из Gazebo Classic."""
import argparse
import re
import time

import rclpy
from gazebo_msgs.srv import DeleteEntity, GetModelList

TRAIL_NAME = re.compile(r"navigation_trail_[0-9]+_[0-9]+_[0-9]+")


def trail_names(names):
    return [name for name in names if TRAIL_NAME.fullmatch(name)]


def request(node, service_type, message, timeout):
    deadline = time.monotonic() + timeout
    type_name = f"gazebo_msgs/srv/{service_type.__name__}"
    service = None
    while rclpy.ok() and time.monotonic() < deadline:
        candidates = sorted(name for name, types in node.get_service_names_and_types()
                            if type_name in types)
        if candidates:
            service = candidates[0]
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if service is None:
        raise RuntimeError(f"Служба {type_name} не найдена. Запустите симулятор Gazebo.")
    client = node.create_client(service_type, service)
    try:
        if not client.wait_for_service(timeout_sec=max(0.0, deadline-time.monotonic())):
            raise TimeoutError(f"Служба {service} недоступна")
        future = client.call_async(message)
        rclpy.spin_until_future_complete(node, future,
                                        timeout_sec=max(0.0, deadline-time.monotonic()))
        if not future.done():
            future.cancel()
            raise TimeoutError(f"Нет ответа от {service}")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"{service}: {getattr(response, 'status_message', 'ошибка')}")
        return response
    finally:
        node.destroy_client(client)


def clear_trails(node, timeout=10.0):
    models = request(node, GetModelList, GetModelList.Request(), timeout)
    names = trail_names(models.model_names)
    deleted = 0
    for name in names:
        message = DeleteEntity.Request()
        message.name = name
        request(node, DeleteEntity, message, timeout)
        deleted += 1
    return deleted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='Тайм-аут каждого запроса Gazebo, с')
    args = parser.parse_args()
    if not 0 < args.timeout < float('inf'):
        parser.error('--timeout должен быть конечным числом больше нуля')
    rclpy.init()
    node = rclpy.create_node('clear_navigation_trails')
    try:
        count = clear_trails(node, args.timeout)
        print(f"Удалено точек следа: {count}. Новые следы остаются включены согласно настройкам навигации.")
        return 0
    except (RuntimeError, TimeoutError) as error:
        print(f"Очистка не завершена: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
