#!/usr/bin/env bash
# Установка ROS 2 Jazzy и зависимостей лабораторной работы на Ubuntu 24.04.
# Запуск: ./SCRIPTS/S1_installRos2.sh  (из любой директории)

set -euo pipefail

ROS_DISTRO=jazzy
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}/ros2_ws"
SRC_DIR="${WORKSPACE_DIR}/src"

. /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "Сценарий рассчитан только на Ubuntu 24.04. Обнаружено: ${PRETTY_NAME:-неизвестная ОС}." >&2
  exit 1
fi

echo "Установка ROS 2 ${ROS_DISTRO^} для Ubuntu ${VERSION_ID}..."
sudo apt update
sudo apt install -y locales software-properties-common curl gnupg lsb-release git \
  python3-paho-mqtt python3-colcon-common-extensions
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

# Официальный репозиторий ROS 2.
# Не завершаем awk после первой строки: при set -o pipefail раннее закрытие
# канала приводит к curl (23), хотя ответ GitHub получен успешно.
ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | awk -F'"' '/"tag_name"/ && !found { print $4; found=1 }')"
[[ -n "${ROS_APT_SOURCE_VERSION}" ]] || { echo "Не удалось определить версию ros2-apt-source." >&2; exit 1; }
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Gazebo Harmonic нужен симулятору TurtleBot3 в ветке Jazzy.
# Сначала скачиваем ключ в доступный пользователю временный файл: это исключает
# ошибку curl (23) в системах, где sudo запускает команды в отдельном PTY.
GAZEBO_KEY=/tmp/pkgs-osrf-archive-keyring.gpg
curl -fsSL -o "${GAZEBO_KEY}" https://packages.osrfoundation.org/gazebo.gpg
sudo install -m 0644 "${GAZEBO_KEY}" /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${VERSION_CODENAME} main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-desktop ros-dev-tools \
  ros-${ROS_DISTRO}-cartographer ros-${ROS_DISTRO}-cartographer-ros \
  ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup gz-harmonic

mkdir -p "${SRC_DIR}"
clone_if_missing() {
  local repository="$1" directory="$2"
  if [[ ! -d "${SRC_DIR}/${directory}/.git" ]]; then
    if [[ -e "${SRC_DIR}/${directory}" ]]; then
      echo "${SRC_DIR}/${directory} существует, но не похож на исходный пакет. Удалите или исправьте его вручную." >&2
      exit 1
    fi
    git clone --branch "${ROS_DISTRO}" --depth 1 "${repository}" "${SRC_DIR}/${directory}"
  fi
}

# Бинарные пакеты не содержат всех миров симуляции из лабораторной работы.
clone_if_missing https://github.com/ROBOTIS-GIT/DynamixelSDK.git DynamixelSDK
clone_if_missing https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git turtlebot3_msgs
clone_if_missing https://github.com/ROBOTIS-GIT/turtlebot3.git turtlebot3
clone_if_missing https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git turtlebot3_simulations

# Стандартный setup.bash ROS обращается к необъявленным переменным; временно
# отключаем nounset, сохраняя строгий режим для остального сценария.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
if ! sudo rosdep init 2>/dev/null; then echo "rosdep уже инициализирован."; fi
rosdep update
rosdep install --from-paths "${SRC_DIR}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y

cd "${WORKSPACE_DIR}"
colcon build --symlink-install --executor sequential

echo
echo "Установка завершена. Для нового терминала:"
echo "  source /opt/${ROS_DISTRO}/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
python3 -c "import paho.mqtt.client; print('paho-mqtt: OK')"
echo "ROS 2: ${ROS_DISTRO}"
