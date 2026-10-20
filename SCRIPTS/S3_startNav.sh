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
NAVIGATION_MODE="${TB3_NAVIGATION_MODE:-position}"
NAV_ARGUMENTS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      [[ $# -ge 2 ]] || { echo "После --mode требуется position, orientation или proximity" >&2; exit 2; }
      NAVIGATION_MODE="$2"; shift 2 ;;
    --orientation) NAVIGATION_MODE=orientation; shift ;;
    --ignore-orientation) NAVIGATION_MODE=position; shift ;;
    *) NAV_ARGUMENTS+=("$1"); shift ;;
  esac
done
case "${NAVIGATION_MODE}" in
  position|orientation|proximity) ;;
  *) echo "Неверный --mode: ${NAVIGATION_MODE}" >&2; exit 2 ;;
esac
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
echo "Прибытие по умолчанию: ${NAVIGATION_MODE}"
# RViz goals use the same default as the direct and MQTT senders. Each MQTT
# Point can still override it with Mode. Keep the installed YAML untouched.
RUNTIME_PARAMS="$(mktemp --suffix=.yaml lw2_nav_XXXXXX --tmpdir)"
NAV_PID=""
cleanup() {
  if [[ -n "${NAV_PID}" ]]; then
    kill -INT "${NAV_PID}" 2>/dev/null || true
    wait "${NAV_PID}" 2>/dev/null || true
  fi
  rm -f "${RUNTIME_PARAMS}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 - "${PARAMS_FILE}" "${RUNTIME_PARAMS}" "${SCRIPT_DIR}" "${NAVIGATION_MODE}" <<'PY'
import sys
from pathlib import Path
import yaml
sys.path.insert(0, sys.argv[3])
from navigation_modes import behavior_tree
params = yaml.safe_load(Path(sys.argv[1]).read_text())
params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = behavior_tree(sys.argv[4])
Path(sys.argv[2]).write_text(yaml.safe_dump(params, sort_keys=False))
PY
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:="${MAP_FILE}" \
  params_file:="${RUNTIME_PARAMS}" \
  "${NAV_ARGUMENTS[@]}" &
NAV_PID=$!
wait "${NAV_PID}"
NAV_PID=""
