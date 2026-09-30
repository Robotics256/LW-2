#!/usr/bin/env bash
# Запуск Navigation2 с явно указанной сохранённой картой.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO=humble

if [[ $# -lt 1 ]]; then
  echo "Использование: $0 ПУТЬ_К_КАРТЕ.yaml" >&2
  echo "Пример: $0 ~/warehouse_map.yaml" >&2
  exit 2
fi

MAP_FILE="$1"
shift
if [[ ! -f "${MAP_FILE}" ]]; then
  echo "Карта не найдена: ${MAP_FILE}" >&2
  exit 1
fi
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO^} не найден. Сначала запустите S1_installRos2.sh." >&2
  exit 1
fi
if [[ ! -f "${PROJECT_DIR}/ros2_ws/install/setup.bash" ]]; then
  echo "Workspace не собран. Сначала запустите S1_installRos2.sh." >&2
  exit 1
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
# RViz/OGRE can freeze on the hardware OpenGL path provided by WSLg. Use
# Mesa's software renderer by default; set RVIZ_SOFTWARE_RENDERING=0 to test
# the hardware renderer after updating WSL and the GPU driver.
if [[ "${RVIZ_SOFTWARE_RENDERING:-1}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  echo "RViz: используется программный OpenGL (RVIZ_SOFTWARE_RENDERING=0 для аппаратного режима)"
fi
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
set -u

PARAMS_FILE="$(ros2 pkg prefix --share turtlebot3_gazebo)/params/warehouse_${TURTLEBOT3_MODEL}.yaml"
if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "Параметры Nav2 не найдены: ${PARAMS_FILE}" >&2
  exit 1
fi

echo "Карта Nav2: ${MAP_FILE}"
exec ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:="${MAP_FILE}" \
  params_file:="${PARAMS_FILE}" \
  "$@"
