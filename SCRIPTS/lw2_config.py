"""Load the shared user configuration for LW-2 scripts."""

import json
import math
import os
from pathlib import Path


DEFAULT_CONFIG_FILE = Path(__file__).with_name("lw2_config.json")
CARGO_OPTIONS = ("floor_markers_enabled", "robot_color_enabled")

# All publishers and the driver use the same address and robot identity.
WORKSPACE = os.environ.get("TB3_MQTT_WORKSPACE", "space_v21")
MODULE_NAME = os.environ.get("TB3_MQTT_NAME", "amr1_v21")
MODULE_TYPE = "delivery"
MQTT_BROKER = os.environ.get("TB3_MQTT_BROKER", "broker.emqx.io")
MQTT_PORT = int(os.environ.get("TB3_MQTT_PORT", "1883"))
MQTT_COMMAND_TOPIC = f"{WORKSPACE}/devices/in/{MODULE_TYPE}/{MODULE_NAME}"
MQTT_STATUS_TOPIC = f"{WORKSPACE}/devices/out/{MODULE_TYPE}/{MODULE_NAME}"


def configure_mqtt_auth(client):
    username = os.environ.get("TB3_MQTT_USERNAME")
    if username:
        client.username_pw_set(username, os.environ.get("TB3_MQTT_PASSWORD", ""))


def load_project_config(path=DEFAULT_CONFIG_FILE):
    """Read the common JSON object used by the project's Python scripts."""
    config_path = Path(path).expanduser()
    try:
        data = json.loads(config_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"Не удалось прочитать конфиг {config_path}: {error}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"Некорректный JSON в {config_path}: {error}") from error

    if not isinstance(data, dict):
        raise ValueError(f"Конфиг {config_path} должен содержать JSON-объект")
    return data


def load_cargo_config(path=DEFAULT_CONFIG_FILE):
    """Load and strictly validate the two independent cargo visual flags."""
    config_path = Path(path).expanduser()
    data = load_project_config(config_path)
    cargo = data.get("cargo")
    if not isinstance(cargo, dict):
        raise ValueError(f"Раздел 'cargo' отсутствует в {config_path}")

    result = {}
    for option in CARGO_OPTIONS:
        value = cargo.get(option)
        if not isinstance(value, bool):
            raise ValueError(
                f"Параметр 'cargo.{option}' в {config_path} должен быть true или false"
            )
        result[option] = value
    return result


def load_teleop_config(path=DEFAULT_CONFIG_FILE):
    """Maximum absolute keyboard commands, in m/s and rad/s."""
    config = load_project_config(path).get('teleop')
    if not isinstance(config, dict):
        raise ValueError(f"Раздел 'teleop' отсутствует в {path}")
    result = {}
    for name in ('max_linear_speed', 'max_angular_speed'):
        value = config.get(name)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0):
            raise ValueError(f'teleop.{name} должен быть конечным неотрицательным числом')
        result[name] = float(value)
    return result


def load_trail_color(color_number, path=DEFAULT_CONFIG_FILE):
    """Return one RGB trail colour from the shared configuration."""
    if color_number is None:
        return None

    config_path = Path(path).expanduser()
    data = load_project_config(config_path)
    colors = data.get("trail_colors")
    if not isinstance(colors, dict):
        raise ValueError(f"Раздел 'trail_colors' отсутствует в {config_path}")
    try:
        color = colors[str(color_number)]
    except KeyError as error:
        raise ValueError(
            f"Цвет следа {color_number} отсутствует в {config_path}"
        ) from error

    if not (
        isinstance(color, list)
        and len(color) == 3
        and all(
            not isinstance(component, bool)
            and isinstance(component, (int, float))
            and 0 <= component <= 1
            for component in color
        )
    ):
        raise ValueError("Цвет следа должен быть списком из трёх чисел от 0 до 1")
    return tuple(color)
