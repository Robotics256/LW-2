#!/usr/bin/env bash
# Установка ROS 2 Humble и зависимостей лабораторной работы на Ubuntu 22.04.
# Запуск: ./SCRIPTS/S1_installRos2.sh (из любой директории)

set -euo pipefail

ROS_DISTRO=humble
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}/ros2_ws"

. /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "Сценарий рассчитан на Ubuntu 22.04. Обнаружено: ${PRETTY_NAME:-неизвестная ОС}." >&2
  exit 1
fi

sudo apt update
sudo apt install -y locales software-properties-common curl git
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe

# Официальный репозиторий пакетов ROS 2.
ROS_APT_SOURCE_VERSION="$(
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    awk -F'"' '/"tag_name"/ && !found { print $4; found=1 }'
)"
if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
  echo "Не удалось определить версию ros2-apt-source." >&2
  exit 1
fi
ROS_APT_DEB="/tmp/ros2-apt-source-${ROS_APT_SOURCE_VERSION}.deb"
curl -fsSL -o "${ROS_APT_DEB}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
sudo dpkg -i "${ROS_APT_DEB}"

sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-dev-tools python3-colcon-common-extensions \
  python3-paho-mqtt python3-yaml \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-cartographer ros-humble-cartographer-ros \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-nav2-collision-monitor

set +u
source /opt/ros/humble/setup.bash
set -u

if ! sudo rosdep init 2>/dev/null; then
  echo "rosdep уже инициализирован."
fi
rosdep update
rosdep install --from-paths "${WORKSPACE_DIR}/src" --ignore-src \
  --rosdistro humble -r -y

echo "=========================================="
echo "Сборка workspace ROS 2 Humble"
echo "=========================================="
cd "${WORKSPACE_DIR}"
# Recreate ignored reference maps and all three navigation checkers on a fresh checkout.
python3 "${SCRIPT_DIR}/generate_warehouse.py"
colcon build --symlink-install --executor sequential

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

echo "=========================================="
echo "Проверка установки ROS 2 Humble + TurtleBot3"
echo "=========================================="
python3 --version
python3 -c "import paho.mqtt.client; print('paho-mqtt: OK')"
python3 -c "import yaml; from nav2_simple_commander.robot_navigator import BasicNavigator; print('YAML, Nav2 commander: OK')"
WAREHOUSE_SHARE="$(ros2 pkg prefix --share turtlebot3_gazebo)"
for asset in worlds/turtlebot3_warehouse_automotive.world maps/warehouse_automotive.yaml maps/warehouse_automotive.pgm; do
  [[ -f "${WAREHOUSE_SHARE}/${asset}" ]] || { echo "Не установлен ресурс ${asset}" >&2; exit 1; }
done
echo "ROS 2 дистрибутив: ${ROS_DISTRO}"
echo "ROS 2: $(command -v ros2)"
echo "Workspace: ${WORKSPACE_DIR}"
echo "Установка завершена."
