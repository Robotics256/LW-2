#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from waypoint_markers import WaypointMarkerPublisher
from waypoint_file import load_waypoints

import paho.mqtt.client as mqtt
import json
import os
import random
import queue
import threading
import time
from enum import IntEnum
import uuid

# ================== CONFIG ==================
WAYPOINTS_FILE = os.environ.get("TB3_POINTS_FILE", os.path.expanduser("~/points.txt"))

WORKSPACE = "space_v21"     # переименуйте в соответствии с вашим вариантом
MODULE_NAME = "amr1_v21"    # переименуйте в соответствии с вашим вариантом
MODULE_TYPE = "delivery"    

MQTT_BROKER = "broker.emqx.io"  # тестовый публичный брокер 
MQTT_PORT = 1883
MQTT_SUBSCRIBE_TOPIC = f"{WORKSPACE}/devices/in/{MODULE_TYPE}/{MODULE_NAME}"
MQTT_PUBLISH_TOPIC = f"{WORKSPACE}/devices/out/{MODULE_TYPE}/{MODULE_NAME}"

# при использовании приватного брокера установить имя и пароль и расскоментировать строку 95
# MQTT_USERNAME = "username" 
# MQTT_PASSWORD = "password"
# ============================================

class TaskStatus(IntEnum):
    DONE = 0         # успешное выполнение
    IN_PROGRESS = 1  # задание в стадии выполнения  
    ERROR = 2        # ошибка выполнения
    BUSY = 3         # новое задание не принято, робот занят выполнением другого задания

class MqttNavNode(Node):

    def __init__(self):
        super().__init__("mqtt_nav_node")

        # Nav2
        self.navigator = BasicNavigator()
        self.waypoints = load_waypoints(WAYPOINTS_FILE)
        for waypoint in self.waypoints:
            print(f"Loaded waypoint '{waypoint['name']}'")
        self.waypoint_marker_publisher = WaypointMarkerPublisher(self)
        self.waypoint_marker_publisher.publish(self.waypoints)

        self.get_logger().info(f"Загружено точек: {len(self.waypoints)}")
        self.navigator.waitUntilNav2Active()

        # Очередь заданий
        self.task_queue = queue.Queue()
        self.current_goal = None

        # Текущее и выполненные задания
        self.current_task_id = None
        self.current_task_start = None  # time.time()
        self.done_tasks = {}

        # MQTT
        self.get_logger().info("Инициализация MQTT")
        self.mqtt_client = mqtt.Client(
            client_id=f"mqtt-nav-{random.randint(0, 10000)}"
        )
        # self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

        threading.Thread(
            target=self.mqtt_client.loop_forever,
            daemon=True
        ).start()

        # Таймер — главный цикл навигации
        self.create_timer(0.1, self.navigation_loop)

    # ================= MQTT ==================
    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("MQTT подключён")
        client.subscribe(MQTT_SUBSCRIBE_TOPIC)
        self.get_logger().info(f"Подписан на {MQTT_SUBSCRIBE_TOPIC}")

    def on_message(self, client, userdata, msg):
        try:
            root = json.loads(msg.payload.decode())

            module = root.get("Module", {})
            data = module.get("Data", {})
            task = data.get("Task", {})

            task_type = task.get("Type")
            task_data = task.get("Data")
            task_id = task.get("Id")

            self.get_logger().info(
                f"Получено задание: type={task_type}, data={task_data}, id={task_id}"
            )

            # Обработка только заданий типа "Point"
            if task_type != "Point":
                return

            # Если это задание уже выполнено ранее
            if task_id in self.done_tasks:
                info = self.done_tasks[task_id]
                status = info["status"]          # 0 или 2
                duration = info["end"] - info["start"]
                self.send_task_status(
                    task_id=task_id,
                    status=status,
                    report="task already finished",
                    duration=duration
                )
                return

            # Если это текущее задание
            if task_id == self.current_task_id:
                # 1 — задание на стадии выполнения
                self.send_task_status(
                    task_id=task_id,
                    status=int(TaskStatus.IN_PROGRESS),
                    report="task in progress"
                )
                return

            # Новый TaskId
            # Если робот занят другим заданием
            if self.current_task_id is not None:
                # статус 3, report "robot execute task [номер текущего задания]"
                self.send_task_status(
                    task_id=task_id,
                    status=int(TaskStatus.BUSY),
                    report=f"robot execute task {self.current_task_id}"
                )
                return

            # Робот свободен — принимаем задание
            point_name = task_data.strip()
            target_waypoint = None
            for wp in self.waypoints:
                if wp["name"] == point_name:
                    target_waypoint = wp
                    break

            if target_waypoint is None:
                self.send_task_status(task_id=task_id, status=int(TaskStatus.ERROR), report=f"point '{point_name}' not exists")
                return

            # Сохраняем задачу
            self.current_task_id = task_id
            self.current_task_start = time.time()
            self.task_queue.put(target_waypoint)

            # 1 — задание принято и выполняется
            self.send_task_status(
                task_id=task_id,
                status=int(TaskStatus.IN_PROGRESS),
                report=f"task accepted, go to point {point_name}"
            )

        except Exception as e:
            self.get_logger().error(f"Ошибка обработки MQTT: {e}")

    # ============== ОТПРАВКА СТАТУСА ===============
    def send_task_status(self, task_id, status, report=None, duration=None):
        """
        Если задание выполнено (0) и передано duration,
        она добавляется в ответ.
        """
        try:
            print ("\nsend task status ",status)
            timestamp = int(time.time() * 1000)

            # Базовая структура ответа (можете адаптировать под свой формат)
            payload = {
                "Module": {
                    "Name": MODULE_NAME,
                    "Type": "delivery",
                    "UID":  int(uuid.uuid4().int) % 1000000,
                    "Time": timestamp,
                    "Data": {
                        "Task": {
                            "Id": task_id,
                            "Status": status
                        }
                    }
                }
            }

            if report is not None:
                payload["Module"]["Data"]["Task"]["Report"] = report

            if duration is not None:
                payload["Module"]["Data"]["Task"]["Duration"] = int(duration)

            json_payload = json.dumps(payload)
            # Отправляем в тот же топик (или другой, если у вас отдельный для статусов)
            self.mqtt_client.publish(MQTT_PUBLISH_TOPIC, json_payload, qos=0)
            self.get_logger().info(
                f"Отправлен статус задания id={task_id}, status={status}, report={report}"
            )
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки статуса MQTT: {e}")

    # ============== NAVIGATION ===============
    def navigation_loop(self):
        """
        Вызывается по таймеру.
        Здесь вызывается Nav2.
        """

        # Если нет активной цели — берём новую
        if self.current_goal is None and not self.task_queue.empty():
            waypoint = self.task_queue.get()  # получаем словарь
            target = waypoint["pose"]
            point_name = waypoint["name"]

            self.current_goal = target
            self.get_logger().info(f"Движение к точке '{point_name}': x={target.pose.position.x}")

            self.navigator.goToPose(target)
            return

        # Если цель активна — проверяем статус
        if self.current_goal is not None:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()

                end_time = time.time()
                if self.current_task_start is None:
                    self.current_task_start = end_time
                duration = end_time - self.current_task_start

                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Цель достигнута")
                    # 0 — успешное выполнение
                    if self.current_task_id is not None:
                        self.done_tasks[self.current_task_id] = {
                            "status": int(TaskStatus.DONE),
                            "start": self.current_task_start,
                            "end": end_time
                        }
                        self.send_task_status(
                            task_id=self.current_task_id,
                            status=int(TaskStatus.DONE),
                            report="task completed",
                            duration=duration
                        )
                else:
                    self.get_logger().warn(
                        f"Навигация завершилась с результатом: {result}"
                    )
                    # 2 — ошибка выполнения
                    if self.current_task_id is not None:
                        self.done_tasks[self.current_task_id] = {
                            "status": int(TaskStatus.ERROR),
                            "start": self.current_task_start,
                            "end": end_time
                        }
                        self.send_task_status(
                            task_id=self.current_task_id,
                            status=int(TaskStatus.ERROR),
                            report=f"navigation result: {result}",
                            duration=duration
                        )

                self.current_goal = None
                self.current_task_id = None
                self.current_task_start = None


def main():
    rclpy.init()
    node = MqttNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
