#!/bin/bash  chmod +x ./start_tb3_2.sh

set -e

export TURTLEBOT3_MODEL=waffle
source /home/user/ros2/ros2_ws/install/setup.bash

PIDS=()

cleanup() {
    echo "Stopping TurtleBot3 stack..."
    for pid in "${PIDS[@]}"; do
        kill -SIGINT "$pid" 2>/dev/null
    done
    wait
    exit 0
}

trap cleanup SIGINT SIGTERM

ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4new.launch.py &
PIDS+=($!)

sleep 5

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &
PIDS+=($!)

sleep 5

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=./map.yaml &
PIDS+=($!)

sleep 3

python3 /home/user/ros2/ros2_ws/SKRIPTS/mqttDriver.py &
PIDS+=($!)

wait
