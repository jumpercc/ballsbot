#!/bin/bash

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash
source devel/setup.bash

rm -rf src/ballsbot_detection && \
rm -rf src/ballsbot_laser_ranging_sensor && \
cd src && \
ln -s /home/ballsbot/projects/ballsbot/ballsbot_detection && \
ln -s /home/ballsbot/projects/ballsbot/ballsbot_laser_ranging_sensor && \
cd .. && \

catkin_make

