#!/bin/bash

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash
source devel/setup.bash

rm -rf src/ballsbot_detection && \
cd src && \
ln -s /home/ballsbot/projects/ballsbot/ballsbot_detection && \
cd .. && \
catkin_make

