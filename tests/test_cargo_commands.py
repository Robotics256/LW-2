"""Unit tests for the non-ROS parts of LW-2 cargo commands."""

import json
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "SCRIPTS"
sys.path.insert(0, str(SCRIPTS_DIR))

from lw2_config import (  # noqa: E402
    DEFAULT_CONFIG_FILE,
    load_cargo_config,
    load_project_config,
    load_trail_color,
)
from send_cargo_command import build_cargo_payload  # noqa: E402


class CargoConfigTests(unittest.TestCase):
    def test_default_config_contains_both_sections_and_all_colors(self):
        config = load_project_config(DEFAULT_CONFIG_FILE)
        self.assertIn("cargo", config)
        self.assertEqual(len(config["trail_colors"]), 21)

    def test_loads_independent_flags(self):
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / "cargo.json"
            config_file.write_text(
                json.dumps({
                    "cargo": {
                        "floor_markers_enabled": False,
                        "robot_color_enabled": True,
                    },
                    "trail_colors": {"7": [0.1, 0.2, 0.3]},
                }),
                encoding="utf-8",
            )
            self.assertEqual(
                load_cargo_config(config_file),
                {
                    "floor_markers_enabled": False,
                    "robot_color_enabled": True,
                },
            )

    def test_rejects_non_boolean_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / "cargo.json"
            config_file.write_text(
                json.dumps({
                    "cargo": {
                        "floor_markers_enabled": "yes",
                        "robot_color_enabled": True,
                    },
                    "trail_colors": {"7": [0.1, 0.2, 0.3]},
                }),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_cargo_config(config_file)

    def test_loads_trail_color_from_same_file(self):
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / "lw2.json"
            config_file.write_text(
                json.dumps({
                    "cargo": {
                        "floor_markers_enabled": True,
                        "robot_color_enabled": True,
                    },
                    "trail_colors": {"7": [0.1, 0.2, 0.3]},
                }),
                encoding="utf-8",
            )
            self.assertEqual(load_trail_color(7, config_file), (0.1, 0.2, 0.3))


class CargoPayloadTests(unittest.TestCase):
    def test_load_uses_protocol_upload_type(self):
        task_id, payload = build_cargo_payload("load", task_id=17)
        task = payload["Module"]["Data"]["Task"]
        self.assertEqual(task_id, 17)
        self.assertEqual(task["Id"], 17)
        self.assertEqual(task["Type"], "Upload")

    def test_unload_uses_protocol_download_type(self):
        _, payload = build_cargo_payload("unload", task_id=18)
        self.assertEqual(payload["Module"]["Data"]["Task"]["Type"], "Download")

    def test_rejects_unknown_action(self):
        with self.assertRaises(ValueError):
            build_cargo_payload("move")


if __name__ == "__main__":
    unittest.main()
