#!/bin/bash

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash
source devel/setup.bash

rosrun ballsbot_camera camera_publisher $1
