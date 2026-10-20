#!/usr/bin/env bash
# Запуск симуляции TurtleBot3 после выполнения S1_installRos2.sh.
# По умолчанию: small, navigation (лидар скрыт, сохранённые точки показаны).
# mode:=mapping показывает лидар и скрывает метки точек; /scan работает всегда.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO=humble

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO^} не найден. Сначала запустите S1_installRos2.sh." >&2
  exit 1
fi
if [[ ! -f "${PROJECT_DIR}/ros2_ws/install/setup.bash" ]]; then
  echo "Workspace не собран. Сначала запустите S1_installRos2.sh." >&2
  exit 1
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
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
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
set -u

# Если планировка явно не указана, запускается малый склад.
HAS_WORLD_ARGUMENT=false
for launch_argument in "$@"; do
  if [[ "${launch_argument}" == world:=* || "${launch_argument}" == map:=* ]]; then
    HAS_WORLD_ARGUMENT=true
    break
  fi
done

if [[ "${HAS_WORLD_ARGUMENT}" == true ]]; then
  exec ros2 launch turtlebot3_gazebo turtlebot3_warehouse.launch.py "$@"
else
  exec ros2 launch turtlebot3_gazebo turtlebot3_warehouse.launch.py world:=small "$@"
fi
