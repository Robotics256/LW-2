#!/bin/bash  
# chmod +x ./S1_installRos2.sh

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y

# Установка ROS, RViz, демоверсии, обучающие материалы
sudo apt install -y ros-humble-desktop ros-dev-tools 
# Установка Python3 и pip
sudo apt install -y python3 python3-pip   #python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# Установка paho-mqtt
sudo pip3 install paho-mqtt

# загрузка примеров ROS2
# echo "Загрузка примеров ROS2  "
# mkdir -p ./LW-2/ros2_ws/src
# cd ./LW-2/ros2_ws/src

# git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
# git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

# СБОРКА WORKSPACE 
echo "=========================================="
echo "Сборка ROS2  "
echo "=========================================="
cd ./LW-2/ros2_ws
source /opt/ros/humble/setup.bash

# Инициализация rosdep и сборка
sudo rosdep init || true 
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --executor sequential
source install/setup.bash

# Проверка установки
echo "=========================================="
echo "Проверка установки ROS2 Humble + TurtleBot3"
echo "=========================================="
python3 --version
pip3 --version
python3 -c "import paho.mqtt.client; print('paho-mqtt установлен')" 2>/dev/null || echo "paho-mqtt не найден"
echo "ROS2 дистрибутив: $ROS_DISTRO"
echo "ROS2 путь: $(which ros2)"
ls -la ./LW-2/ros2_ws/src/ | head -4
echo "=========================================="
echo "Установка завершена!"
echo "=========================================="