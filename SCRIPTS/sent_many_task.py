#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import random
import time
import threading




WORKSPACE = "space_v21"
MODULE_NAME = "amr1_v21"
MODULE_TYPE = "delivery"

MQTT_PUBLISH_TOPIC = f"{WORKSPACE}/devices/in/{MODULE_TYPE}/{MODULE_NAME}"
MQTT_SUBSCRIBE_TOPIC = f"{WORKSPACE}/devices/out/{MODULE_TYPE}/{MODULE_NAME}"
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
# MQTT_USERNAME = "username"
# MQTT_PASSWORD = "password"

def get_random_int(max_value):
    return random.randint(1, max_value)


def build_task_payload(point_name: int):
    task_id = get_random_int(100000)
    uid = get_random_int(100000)
    timestamp = int(time.time() * 1000)

    payload = {
        "Module": {
            "Name": MODULE_NAME,
            "ModuleType": MODULE_TYPE,
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


class TaskClient:
    def __init__(self):
        # защита и хранилище статусов
        self._lock = threading.Lock()
        self._task_status = {}
        # событие для пробуждения ждущих потоков
        self._status_event = threading.Event()

        self.client = mqtt.Client(client_id=f"task-sender-{random.randint(0, 10000)}")
        # self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.on_message = self.on_message
        self.client.subscribe(MQTT_SUBSCRIBE_TOPIC)
        self.client.loop_start()

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()

    # ========= приём статусов =========

    def on_message(self, client, userdata, msg):
        try:
            root = json.loads(msg.payload.decode())
            module = root.get("Module", {})
            data = module.get("Data", {})
            status_obj = data.get("Task")

            if not status_obj:
                return

            task_id = status_obj.get("Id")
            status = status_obj.get("Status")
            report = status_obj.get("Report")
            duration = status_obj.get("Duration")

            if task_id is None or status is None:
                return

            with self._lock:
                self._task_status[task_id] = {
                    "status": status,
                    "report": report,
                    "duration": duration,
                }
                # будим всех ожидающих
                self._status_event.set()
        except Exception as e:
            print(f"Ошибка обработки статуса: {e}")

    # ========= отправка задания =========

    def send_move_task(self, point_name) -> int:
        task_id, payload = build_task_payload(point_name)
        json_payload = json.dumps(payload)

        result = self.client.publish(MQTT_PUBLISH_TOPIC, json_payload, qos=2)
        result.wait_for_publish()

        print(f"Отправлено задание Id={task_id} на точку {point_name}")
        return task_id

    # ========= ожидание выполнения =========

    def wait_task_success(self, task_id: int, timeout: float = 120.0):
        """
        Блокирующее ожидание статуса задания.
        Успех: Status == 0 -> возврат (можно вернуть duration/report).
        Ошибка: Status == 2 или 3 -> raise Exception.
        Таймаут: если за timeout секунд нет финального статуса -> raise TimeoutError.
        """
        deadline = time.time() + timeout

        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise TimeoutError(f"Timeout ожидания статуса для Id={task_id}")

            # ждём новое событие статуса
            self._status_event.clear()
            self._status_event.wait(timeout=remaining)

            with self._lock:
                info = self._task_status.get(task_id)

            if not info:
                # ещё нет статуса этого task_id
                continue

            status = info["status"]
            report = info.get("report")
            duration = info.get("duration")

            # 0 — успех
            if status == 0:
                print(
                    f"Задание {task_id} выполнено успешно. "
                    f"duration={duration}, report={report}"
                )
                return info  # можно вернуть словарь

            # 1 — в процессе, ждём дальше
            if status == 1:
                continue

            # 2 / 3 — ошибка, кидаем исключение
            raise Exception(
                f"Ошибка выполнения задания {task_id}: "
                f"status={status}, report={report}"
            )


if __name__ == "__main__":
    client = TaskClient()
    try:
        # 1) отправляем первое задание
        task_id = client.send_move_task("parking")
        # ждём успешного выполнения
        client.wait_task_success(task_id, timeout=300)
   
        # 2) если первое задание выполнено — отправляем второе
        task_id = client.send_move_task("...")
        client.wait_task_success(task_id, timeout=300)
        
    except Exception as e:
        print(f"Ошибка при выполнении задания: {e}")
    finally:
        client.close()
