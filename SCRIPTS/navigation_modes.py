"""Shared navigation defaults for direct Nav2 and JSON MQTT commands."""

from pathlib import Path
from lw2_config import load_navigation_config


DEFAULT_MODE = load_navigation_config()["default_mode"]
MODES = ("position", "orientation", "proximity")
TREES = {
    "position": "position_only_navigation.xml",
    "orientation": "orientation_aware_navigation.xml",
    "proximity": "proximity_navigation.xml",
}


def behavior_tree(mode):
    if mode not in MODES:
        raise ValueError(f"Unknown navigation mode: {mode}")
    return str(Path(__file__).with_name(TREES[mode]))


def add_mode_arguments(parser, default=DEFAULT_MODE):
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--mode", choices=MODES, dest="navigation_mode",
                       help="position: X/Y; orientation: X/Y + сохранённый угол; proximity: радиус из lw2_config.json")
    modes.add_argument("--orientation", dest="navigation_mode", action="store_const",
                       const="orientation", help="Достигать также сохранённой ориентации точки")
    modes.add_argument("--ignore-orientation", dest="navigation_mode", action="store_const",
                       const="position", help="Точное прибытие по X/Y (0.03 м), без конечного доворота")
    parser.set_defaults(navigation_mode=default)


def add_trail_arguments(parser):
    trails = parser.add_mutually_exclusive_group()
    trails.add_argument("--trail", nargs="?", type=int, const=None,
                        help="След включён по умолчанию; необязательный номер цвета 1–21")
    trails.add_argument("--no-trail", dest="trail", action="store_const", const=False,
                        help="Отключить след движения")
    parser.set_defaults(trail=None)


def validate_trail(parser, value):
    if value is not False and value is not None and not 1 <= value <= 21:
        parser.error("--trail принимает номер цвета от 1 до 21")
