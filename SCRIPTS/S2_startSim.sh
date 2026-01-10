#!/bin/bash  
# chmod +x ./S2_startSim.sh
export TURTLEBOT3_MODEL=burger
source ./LW-2/ros2_ws/install/setup.bash 
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py