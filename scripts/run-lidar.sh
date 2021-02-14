#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/projects/ydlidar_ws/devel/setup.bash

cd ~/projects/ydlidar_ws/ && \
roslaunch ydlidar_ros X2L.launch

