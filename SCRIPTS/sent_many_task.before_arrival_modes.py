#!/usr/bin/env python3

# Подключить класс для отправки заданий и ожидания результатов.
from task_client import TaskClient


# Указать адрес и порт MQTT-брокера.
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
# Указать свою рабочую область и имя робота в MQTT.
WORKSPACE = "space_v21"
MODULE_NAME = "amr1_v21"


def main():
    """Полный пример доставки; для аргументов используйте sent_many_task_cli.py."""
    # Подключиться к MQTT и подписаться на результаты заданий.
    client = TaskClient(
        broker=MQTT_BROKER, port=MQTT_PORT,
        workspace=WORKSPACE, module_name=MODULE_NAME,
    )
    try:
        # Отправить робота в точку; сохранить идентификатор задания.
        task_id = client.send_move_task("p1")
        # Дождаться успеха этого задания; timeout задаётся в секундах.
        client.wait_task_success(task_id, timeout=300)

        task_id = client.send_move_task("a2")
        client.wait_task_success(task_id, timeout=300)
        # Отправить грузовую команду: load — загрузка, unload — выгрузка.
        task_id = client.send_cargo_task("load")
        client.wait_task_success(task_id, timeout=30)

        task_id = client.send_move_task("t1")
        client.wait_task_success(task_id, timeout=300)
        task_id = client.send_cargo_task("unload")
        client.wait_task_success(task_id, timeout=30)

        task_id = client.send_move_task("s2")
        client.wait_task_success(task_id, timeout=300)
    # При ошибке не отправлять оставшиеся задания.
    except Exception as e:
        print(f"Ошибка при выполнении задания: {e}")
        return 1
    finally:
        # Закрыть MQTT-соединение и остановить приём сообщений.
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
