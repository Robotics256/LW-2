#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import argparse
import json
import random
import time

from waypoint_file import waypoint_names
from navigation_modes import add_mode_arguments
from lw2_config import (WORKSPACE, MODULE_NAME, MODULE_TYPE, MQTT_BROKER, MQTT_PORT,
                        MQTT_COMMAND_TOPIC, configure_mqtt_auth)




MQTT_TOPIC = MQTT_COMMAND_TOPIC
# MQTT_USERNAME = "username"
# MQTT_PASSWORD = "password"

def get_random_int(max_value):
    # аналог функции getRandomInt из JS (1..max)
    return random.randint(1, max_value)


def build_task_payload(point_name, mode=None):
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
                    "Data": point_name,
                }
            }
        }
    }

    if mode is not None:
        payload["Module"]["Data"]["Task"]["Mode"] = mode
    return task_id, payload


def send_move_task(point_name: str, mode=None):
    client = mqtt.Client(client_id=f"task-sender-{random.randint(0, 10000)}")
    configure_mqtt_auth(client)
    # client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    task_id, payload = build_task_payload(point_name, mode)
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
    add_mode_arguments(parser, default=None)
    arguments = parser.parse_args()
    try:
        if arguments.point not in waypoint_names(arguments.points_file):
            parser.error(f"точка '{arguments.point}' не найдена в {arguments.points_file}")
    except (FileNotFoundError, ValueError) as error:
        parser.error(str(error))
    send_move_task(arguments.point, arguments.navigation_mode)
