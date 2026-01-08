#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import random
import time




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
            "ModuleType": module_type,
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


def send_move_task(point_name: int):
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
    # Пример: отправить роботу команду ехать в точку 3
    send_move_task("parking")