#!/usr/bin/env bash
# Совместный запуск малого склада, Nav2 и MQTT-драйвера.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
set -u

WAREHOUSE_SHARE="$(ros2 pkg prefix --share turtlebot3_gazebo)"
MAP_FILE="${1:-${WAREHOUSE_SHARE}/maps/warehouse_small.yaml}"
PIDS=()

cleanup() {
    echo "Stopping TurtleBot3 stack..."
    for pid in "${PIDS[@]}"; do
        kill -SIGINT "${pid}" 2>/dev/null || true
    done
    wait || true
}

trap cleanup EXIT SIGINT SIGTERM

"${SCRIPT_DIR}/S2_startSim.sh" world:=small &
PIDS+=($!)

sleep 8

"${SCRIPT_DIR}/S3_startNav.sh" "${MAP_FILE}" &
PIDS+=($!)

sleep 3

python3 "${SCRIPT_DIR}/mqtt_driver.py" &
PIDS+=($!)

wait
