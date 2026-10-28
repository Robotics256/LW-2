#!/usr/bin/env bash
# Запуск Navigation2 с явно указанной сохранённой картой.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO=humble

if [[ $# -lt 1 ]]; then
  echo "Использование: $0 ПУТЬ_К_КАРТЕ.yaml [points_file:=ПУТЬ] [--initial-pose X Y YAW]" >&2
  echo "Пример: $0 ~/warehouse_map.yaml" >&2
  exit 2
fi

MAP_FILE="$1"
shift
NAVIGATION_MODE="${TB3_NAVIGATION_MODE:-}"
NAV_ARGUMENTS=()
POINTS_FILE="${TB3_POINTS_FILE:-${HOME}/points.txt}"
INITIAL_POSE=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    points_file:=*) POINTS_FILE="${1#points_file:=}"; shift ;;
    --initial-pose)
      [[ $# -ge 4 ]] || { echo "После --initial-pose нужны X Y YAW (метры, радианы, система map)" >&2; exit 2; }
      INITIAL_POSE=("$2" "$3" "$4"); shift 4 ;;
    --mode)
      [[ $# -ge 2 ]] || { echo "После --mode требуется position, orientation или proximity" >&2; exit 2; }
      NAVIGATION_MODE="$2"; shift 2 ;;
    --orientation) NAVIGATION_MODE=orientation; shift ;;
    --ignore-orientation) NAVIGATION_MODE=position; shift ;;
    *) NAV_ARGUMENTS+=("$1"); shift ;;
  esac
done
if [[ -z "${NAVIGATION_MODE}" ]]; then
  NAVIGATION_MODE="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); from lw2_config import load_navigation_config; print(load_navigation_config()["default_mode"])' "${SCRIPT_DIR}")"
fi
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
MARKERS_PID=""
cleanup() {
  # Each child owns a session, so signal its descendants as well as its launcher.
  # Nav2 can remain in costmap activation waiting for map -> base_link even
  # after SIGINT. Never wait indefinitely, or leave its container behind.
  trap '' INT TERM
  local process_group attempt running
  local -a process_groups=()
  [[ -z "${NAV_PID}" ]] || process_groups+=("${NAV_PID}")
  [[ -z "${MARKERS_PID}" ]] || process_groups+=("${MARKERS_PID}")
  for process_group in "${process_groups[@]}"; do
    kill -INT -- "-${process_group}" 2>/dev/null || true
  done
  for attempt in {1..40}; do
    running=false
    for process_group in "${process_groups[@]}"; do
      if kill -0 -- "-${process_group}" 2>/dev/null; then running=true; fi
    done
    [[ "${running}" == true ]] || break
    sleep 0.2
  done
  for process_group in "${process_groups[@]}"; do
    kill -TERM -- "-${process_group}" 2>/dev/null || true
  done
  for attempt in {1..10}; do
    running=false
    for process_group in "${process_groups[@]}"; do
      if kill -0 -- "-${process_group}" 2>/dev/null; then running=true; fi
    done
    [[ "${running}" == true ]] || break
    sleep 0.2
  done
  for process_group in "${process_groups[@]}"; do
    if kill -0 -- "-${process_group}" 2>/dev/null; then
      echo "Процессы группы ${process_group} не завершились за 10 секунд; принудительная остановка." >&2
      kill -KILL -- "-${process_group}" 2>/dev/null || true
    fi
    wait "${process_group}" 2>/dev/null || true
  done
  rm -f "${RUNTIME_PARAMS}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 - "${PARAMS_FILE}" "${RUNTIME_PARAMS}" "${SCRIPT_DIR}" "${NAVIGATION_MODE}" "${INITIAL_POSE[@]}" <<'PY'
import math
import sys
from pathlib import Path
import yaml
sys.path.insert(0, sys.argv[3])
from navigation_modes import behavior_tree
from lw2_config import apply_navigation_config
from navigation_controller import configure_warehouse_controller
params = yaml.safe_load(Path(sys.argv[1]).read_text())
config = apply_navigation_config(params)
configure_warehouse_controller(params)
print("Контроллер склада: Regulated Pure Pursuit, скорость до 0.15 м/с")
print(f"Радиус прибытия proximity: {config['proximity_radius']} м (lw2_config.json)")
params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = behavior_tree(sys.argv[4])
if len(sys.argv) > 5:
    x, y, yaw = map(float, sys.argv[5:8])
    if not all(map(math.isfinite, (x, y, yaw))):
        raise ValueError('Начальная поза должна содержать конечные числа')
    params['amcl']['ros__parameters'].update(
        set_initial_pose=True, initial_pose={'x': x, 'y': y, 'z': 0.0, 'yaw': yaw})
Path(sys.argv[2]).write_text(yaml.safe_dump(params, sort_keys=False))
PY
# Validate arguments and prepare parameters before stopping an existing run.
python3 "${SCRIPT_DIR}/stop_previous_navigation.py" "${PROJECT_DIR}"
export TB3_NAV_PROJECT_DIR="${PROJECT_DIR}"
if [[ ${#INITIAL_POSE[@]} -gt 0 ]]; then
  echo "Начальная поза AMCL в map: ${INITIAL_POSE[*]}"
else
  echo "В RViz выберите 2D Pose Estimate и протяните стрелку положения и направления. Затем задайте Nav2 Goal."
fi
if [[ -f "${POINTS_FILE}" ]]; then
  setsid python3 "${SCRIPT_DIR}/show_waypoints.py" "${POINTS_FILE}" --rviz-only &
  MARKERS_PID=$!
else
  echo "Файл точек не найден: ${POINTS_FILE}; укажите points_file:=ПУТЬ" >&2
fi
setsid ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:="${MAP_FILE}" \
  params_file:="${RUNTIME_PARAMS}" \
  "${NAV_ARGUMENTS[@]}" &
NAV_PID=$!
wait "${NAV_PID}"
