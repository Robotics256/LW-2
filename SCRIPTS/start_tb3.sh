#!/usr/bin/env bash
# Совместный запуск выбранного мира, Nav2 и JSON MQTT-драйвера.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORLD=small
MAP_FILE=""
POINTS_FILE="${TB3_POINTS_FILE:-${HOME}/points.txt}"
NAVIGATION_MODE="${TB3_NAVIGATION_MODE:-}"
GUI=true
TRAIL_ARGUMENTS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --world|--points-file|--mode)
      [[ $# -ge 2 ]] || { echo "Не задано значение $1" >&2; exit 2; }
      case "$1" in
        --world) WORLD="$2" ;;
        --points-file) POINTS_FILE="$2" ;;
        --mode) NAVIGATION_MODE="$2" ;;
      esac
      shift 2 ;;
    --trail)
      [[ $# -ge 2 ]] || { echo "После --trail нужен номер варианта 1–21" >&2; exit 2; }
      [[ "$2" =~ ^[0-9]+$ && ${#2} -le 2 ]] || { echo "Номер варианта должен быть от 1 до 21" >&2; exit 2; }
      (( 10#$2 >= 1 && 10#$2 <= 21 )) || { echo "Номер варианта должен быть от 1 до 21" >&2; exit 2; }
      TRAIL_ARGUMENTS=(--trail "$2"); shift 2 ;;
    --no-trail) TRAIL_ARGUMENTS=(--no-trail); shift ;;
    --orientation) NAVIGATION_MODE=orientation; shift ;;
    --headless) GUI=false; shift ;;
    -h|--help)
      echo "Использование: $0 [карта.yaml] [--world small|large|automotive|stage4] [--points-file путь] [--mode position|orientation|proximity] [--headless] [--trail НОМЕР_ВАРИАНТА | --no-trail]"
      exit 0 ;;
    -*) echo "Неизвестный параметр: $1" >&2; exit 2 ;;
    *) [[ -z "${MAP_FILE}" ]] || { echo "Лишний аргумент: $1" >&2; exit 2; }
       MAP_FILE="$1"; shift ;;
  esac
done
case "${WORLD}" in small|large|automotive|stage4) ;; *) echo "Неверный мир: ${WORLD}" >&2; exit 2 ;; esac
if [[ -z "${NAVIGATION_MODE}" ]]; then
  NAVIGATION_MODE="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); from lw2_config import load_navigation_config; print(load_navigation_config()["default_mode"])' "${SCRIPT_DIR}")"
fi
case "${NAVIGATION_MODE}" in position|orientation|proximity) ;; *) echo "Неверный режим: ${NAVIGATION_MODE}" >&2; exit 2 ;; esac
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export TB3_POINTS_FILE="${POINTS_FILE}"
[[ -f /opt/ros/humble/setup.bash && -f "${PROJECT_DIR}/ros2_ws/install/setup.bash" ]] || {
  echo "Сначала выполните S1_installRos2.sh" >&2; exit 1;
}
[[ -f "${POINTS_FILE}" ]] || { echo "Файл точек не найден: ${POINTS_FILE}" >&2; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
set -u
WAREHOUSE_SHARE="$(ros2 pkg prefix --share turtlebot3_gazebo)"
if [[ -z "${MAP_FILE}" ]]; then
  case "${WORLD}" in
    small) MAP_FILE="${WAREHOUSE_SHARE}/maps/warehouse_small.yaml" ;;
    large) MAP_FILE="${WAREHOUSE_SHARE}/maps/warehouse.yaml" ;;
    automotive) MAP_FILE="${WAREHOUSE_SHARE}/maps/warehouse_automotive.yaml" ;;
    stage4) echo "Для stage4 передайте сохранённую карту.yaml первым аргументом" >&2; exit 2 ;;
  esac
fi
[[ -f "${MAP_FILE}" ]] || { echo "Карта не найдена: ${MAP_FILE}" >&2; exit 1; }
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"${SCRIPT_DIR}/S2_startSim.sh" world:="${WORLD}" mode:=navigation gui:="${GUI}" &
PIDS+=($!)
sleep 8
"${SCRIPT_DIR}/S3_startNav.sh" "${MAP_FILE}" --mode "${NAVIGATION_MODE}" use_rviz:="${GUI}" &
PIDS+=($!)
sleep 3
python3 "${SCRIPT_DIR}/mqtt_driver.py" --points-file "${POINTS_FILE}" --mode "${NAVIGATION_MODE}" "${TRAIL_ARGUMENTS[@]}" &
PIDS+=($!)
wait -n "${PIDS[@]}"
