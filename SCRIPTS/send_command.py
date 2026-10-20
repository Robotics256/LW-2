#!/usr/bin/env python3
"""Send status/history/Stop using LW-2's JSON envelope and wait for a reply."""

import argparse
import json
import threading
import time
import uuid

import paho.mqtt.client as mqtt
from lw2_config import (MODULE_NAME, MODULE_TYPE, MQTT_BROKER, MQTT_PORT,
                        MQTT_COMMAND_TOPIC, MQTT_STATUS_TOPIC, configure_mqtt_auth)
from mqtt_tasks import task_id


def build_command_payload(command, limit=None, identifier=None):
    command = command.lower()
    if identifier is not None:
        if command != "stop":
            raise ValueError("--task-id applies only to stop")
        identifier = task_id(identifier)
    uid = uuid.uuid4().int & ((1 << 63) - 1)
    if command in ("status", "history"):
        data = {"Request": command}
        if limit is not None:
            if command != "history" or isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
                raise ValueError("--limit requires history and a positive integer")
            data["Limit"] = limit
    elif command == "stop":
        data = {"Task": {"Id": identifier if identifier is not None else uid,
                         "Type": "Stop", "Data": ""}}
    else:
        raise ValueError("command must be status, history or stop")
    return {"Module": {"Name": MODULE_NAME, "Type": MODULE_TYPE, "UID": uid,
                       "Time": int(time.time() * 1000), "Data": data}}


def send_command(payload, timeout=15):
    subscribed, completed = threading.Event(), threading.Event()
    replies = []
    module = payload["Module"]
    identifier = module["Data"].get("Task", {}).get("Id")

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(MQTT_STATUS_TOPIC, qos=1)

    def on_message(client, userdata, message):
        try:
            response = json.loads(message.payload.decode("utf-8"))["Module"]
            if response.get("Name") != module["Name"]:
                return
            data = response.get("Data", {})
            if identifier is not None:
                task = data.get("Task", {})
                if task.get("Id") != identifier or task.get("Status") == 1:
                    return
            elif data.get("RequestUID") != module["UID"]:
                return
            replies.append(response)
            completed.set()
        except (ValueError, KeyError, TypeError):
            return

    client = mqtt.Client(client_id=f"lw2-command-{uuid.uuid4().hex[:12]}")
    configure_mqtt_auth(client)
    client.on_connect = on_connect
    client.on_subscribe = lambda *args: subscribed.set()
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
    try:
        if not subscribed.wait(timeout):
            raise TimeoutError("MQTT subscription acknowledgement timed out")
        client.publish(MQTT_COMMAND_TOPIC, json.dumps(payload), qos=1, retain=False).wait_for_publish()
        if not completed.wait(timeout):
            raise TimeoutError("MQTT reply timed out")
        response = replies[-1]
        print(json.dumps({"Module": response}, indent=2, ensure_ascii=False))
        data = response["Data"]
        return 1 if "RequestError" in data or data.get("Task", {}).get("Status", 0) != 0 else 0
    finally:
        client.disconnect()
        client.loop_stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", type=str.lower, choices=("status", "history", "stop"))
    parser.add_argument("--limit", type=int)
    parser.add_argument("--task-id", type=int)
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    try:
        return send_command(build_command_payload(args.command, args.limit, args.task_id), args.timeout)
    except (ValueError, OSError, TimeoutError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
