#!/usr/bin/env python3
"""Send a mixed sequence of waypoint names, up (load) and down (unload)."""

import argparse
from navigation_modes import add_mode_arguments
from task_client import TaskClient
from lw2_config import MQTT_BROKER, MQTT_PORT, WORKSPACE, MODULE_NAME
from waypoint_file import waypoint_names

CARGO_COMMANDS = {"up": "load", "down": "unload"}


def parse_steps(tokens, points_file):
    """Validate every point before connecting; up/down are reserved tokens."""
    points = [token for token in tokens if token not in CARGO_COMMANDS]
    if points:
        known = waypoint_names(points_file)
        missing = [point for point in points if point not in known]
        if missing:
            raise ValueError(f"точки не найдены в {points_file}: {', '.join(missing)}")
    return [("cargo", CARGO_COMMANDS[token]) if token in CARGO_COMMANDS
            else ("point", token) for token in tokens]


def run_sequence(client, steps, mode=None):
    for kind, value in steps:
        if kind == "point":
            task_id = client.send_move_task(value, mode)
            timeout = 300
        else:
            task_id = client.send_cargo_task(value)
            timeout = 30
        client.wait_task_success(task_id, timeout=timeout)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Последовательность MQTT: имена точек, up — загрузка, down — выгрузка."
    )
    parser.add_argument("steps", nargs="+", help="Например: S2 T1 up T2 down S2")
    parser.add_argument("--points-file", default="~/points.txt")
    parser.add_argument("--broker", default=MQTT_BROKER, help="Адрес MQTT-брокера")
    parser.add_argument("--port", type=int, default=MQTT_PORT, help="Порт MQTT-брокера")
    parser.add_argument("--workspace", default=WORKSPACE, help="Рабочая область MQTT")
    parser.add_argument("--name", default=MODULE_NAME, help="Имя робота в MQTT")
    add_mode_arguments(parser, default=None)
    arguments = parser.parse_args(argv)
    try:
        steps = parse_steps(arguments.steps, arguments.points_file)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    client = None
    try:
        client = TaskClient(
            broker=arguments.broker, port=arguments.port,
            workspace=arguments.workspace, module_name=arguments.name,
        )
        run_sequence(client, steps, arguments.navigation_mode)
    except Exception as error:
        print(f"Последовательность прервана: {error}")
        return 1
    finally:
        if client is not None:
            client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
