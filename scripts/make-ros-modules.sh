#!/bin/bash

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash
source devel/setup.bash

rm -rf src/ballsbot_detection && \
rm -rf src/ballsbot_laser_ranging_sensor && \
rm -rf src/ballsbot_tca9548 && \
rm -rf src/ballsbot_magnetic_encoder && \
cd src && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_detection && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_laser_ranging_sensor && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_tca9548 && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_magnetic_encoder && \
cd .. && \

catkin_make

