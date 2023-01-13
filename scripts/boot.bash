#!/bin/bash
cd /home/labbot/labbot_ws
source devel/setup.bash
export ROS_IP=10.42.0.1
export ROS_MASTER_URI=http://labbotJerry:11311
export ROS_HOSTNAME=labbotJerry
roslaunch labbot labbot.launch
