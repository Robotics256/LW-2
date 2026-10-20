#!/usr/bin/env python3
"""Manually activate robot unloading through MQTT."""

from send_cargo_command import main


if __name__ == "__main__":
    main("unload")
