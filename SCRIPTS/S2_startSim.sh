#!/usr/bin/env bash
# Запуск симуляции TurtleBot3 после выполнения S1_installRos2.sh.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO=jazzy

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO^} не найден. Сначала запустите S1_installRos2.sh." >&2
  exit 1
fi
if [[ ! -f "${PROJECT_DIR}/ros2_ws/install/setup.bash" ]]; then
  echo "Workspace не собран. Сначала запустите S1_installRos2.sh." >&2
  exit 1
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
# export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
# File in the common waypoint format:
# name,map_x,map_y,qz,qw[,gazebo_x,gazebo_y].  The warehouse launch reads it
# and creates visual waypoint markers as soon as Gazebo is ready.
POINTS_FILE="${TB3_POINTS_FILE:-${HOME}/points.txt}"
for launch_argument in "$@"; do
  if [[ "${launch_argument}" == points_file:=* ]]; then
    POINTS_FILE="${launch_argument#points_file:=}"
  fi
done
export TB3_POINTS_FILE="${POINTS_FILE}"
export TB3_PROJECT_DIR="${PROJECT_DIR}"
if [[ -f "${POINTS_FILE}" ]]; then
  echo "Точки для отображения: ${POINTS_FILE}"
else
  echo "Файл точек не найден: ${POINTS_FILE}; метки не будут показаны." >&2
fi
# Each warehouse instance must use its own Gazebo Transport partition. Otherwise
# a GUI started after an interrupted run can discover old servers and receive
# duplicate scene entities, which crashes the Gazebo GUI.
if [[ -z "${GZ_PARTITION:-}" ]]; then
  export GZ_PARTITION="tb3_warehouse_${USER:-user}_$$_${RANDOM}"
fi
# Teleop is usually started from another terminal. Give it the same private
# Gazebo Transport partition so it can add visual waypoint models.
PARTITION_FILE="${TMPDIR:-/tmp}/tb3_warehouse_${USER:-user}.gz_partition"
printf '%s\n' "${GZ_PARTITION}" > "${PARTITION_FILE}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
set -u
# В Jazzy Gazebo Sim не получает этот каталог автоматически из ament-окружения.
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PROJECT_DIR}/ros2_ws/install/turtlebot3_gazebo/lib/turtlebot3_gazebo${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
exec ros2 launch turtlebot3_gazebo turtlebot3_warehouse.launch.py "$@"
