#!/bin/bash

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash
source devel/setup.bash

roslaunch ballsbot_main $1
