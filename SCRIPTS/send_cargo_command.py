#!/usr/bin/env python3
"""Manually publish a load/unload task to the LW-2 MQTT driver."""

import argparse
import json
import random
import time

import paho.mqtt.client as mqtt
from lw2_config import (WORKSPACE, MODULE_NAME, MODULE_TYPE, MQTT_BROKER, MQTT_PORT,
                        MQTT_COMMAND_TOPIC, configure_mqtt_auth)


MQTT_TOPIC = MQTT_COMMAND_TOPIC


def build_cargo_payload(action, task_id=None):
    """Build the same JSON task envelope used by the navigation scripts."""
    normalized = action.strip().lower()
    task_types = {
        "load": "Upload",
        "upload": "Upload",
        "unload": "Download",
        "download": "Download",
    }
    if normalized not in task_types:
        raise ValueError("action must be load/upload or unload/download")

    if task_id is None:
        task_id = random.randint(1, 100000)
    payload = {
        "Module": {
            "Name": MODULE_NAME,
            "Type": MODULE_TYPE,
            "UID": random.randint(1, 100000),
            "Time": int(time.time() * 1000),
            "Data": {
                "Task": {
                    "Id": task_id,
                    "Type": task_types[normalized],
                    "Data": "",
                }
            },
        }
    }
    return task_id, payload


def send_cargo_command(action):
    task_id, payload = build_cargo_payload(action)
    client = mqtt.Client(client_id=f"cargo-sender-{random.randint(0, 10000)}")
    configure_mqtt_auth(client)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
    result = client.publish(MQTT_TOPIC, json.dumps(payload), qos=2)
    result.wait_for_publish()
    client.loop_stop()
    client.disconnect()
    print(f"Отправлена команда {payload['Module']['Data']['Task']['Type']}, TaskId={task_id}")


def main(default_action=None):
    parser = argparse.ArgumentParser(description="Send an LW-2 cargo command through MQTT.")
    if default_action is None:
        parser.add_argument("action", choices=("load", "upload", "unload", "download"))
        arguments = parser.parse_args()
        action = arguments.action
    else:
        parser.parse_args()
        action = default_action
    send_cargo_command(action)


if __name__ == "__main__":
    main()
