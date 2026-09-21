#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import argparse
import json
import random
import time

from waypoint_file import waypoints_by_name




WORKSPACE = "space_v21"
MODULE_NAME = "amr1_v21"
MODULE_TYPE = "delivery"

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_TOPIC = f"{WORKSPACE}/devices/in/{MODULE_TYPE}/{MODULE_NAME}"
# MQTT_USERNAME = "username"
# MQTT_PASSWORD = "password"

def get_random_int(max_value):
    # аналог функции getRandomInt из JS (1..max)
    return random.randint(1, max_value)


def build_task_payload(point_name):
    """
    Формирует JSON:
    """
    task_id = get_random_int(100000)
    uid = get_random_int(100000)
    timestamp = int(time.time() * 1000)  # миллисекунды, как Date.now() в JS

    module_name = MODULE_NAME
    module_type = MODULE_TYPE

    payload = {
        "Module": {
            "Name": module_name,
            "Type": module_type,
            "UID": uid,
            "Time": timestamp,
            "Data": {
                "Task": {
                    "Id": task_id,
                    "Type": "Point",
                    "Data": point_name
                }
            }
        }
    }

    return task_id, payload


def send_move_task(point_name: str):
    client = mqtt.Client(client_id=f"task-sender-{random.randint(0, 10000)}")
    # client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    task_id, payload = build_task_payload(point_name)
    json_payload = json.dumps(payload)

    # Публикуем в JSON‑топик (его слушает ваш узел)
    result = client.publish(MQTT_TOPIC, json_payload, qos=0)

    # Можно подождать подтверждения отправки
    result.wait_for_publish()

    client.loop_stop()
    client.disconnect()

    print(f"Отправлено задание TaskId={task_id} на точку {point_name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send one Nav2 waypoint through MQTT.")
    parser.add_argument("point", help="Waypoint name from points.txt")
    parser.add_argument("--points-file", default="~/points.txt", help="Path to points.txt")
    arguments = parser.parse_args()
    try:
        if arguments.point not in waypoints_by_name(arguments.points_file):
            parser.error(f"точка '{arguments.point}' не найдена в {arguments.points_file}")
    except (FileNotFoundError, ValueError) as error:
        parser.error(str(error))
    send_move_task(arguments.point)
