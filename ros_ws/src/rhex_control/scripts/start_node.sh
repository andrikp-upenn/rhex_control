#!/bin/bash
exec &>> /tmp/rover_service.log
source /home/rhex/.bashrc
source /opt/ros/humble/setup.bash
source /home/rhex/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run rover_control rover_node --rover_number 0
