#!/bin/bash  
# chmod +x ./S2_startSim.sh

cd ~/LR2/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py