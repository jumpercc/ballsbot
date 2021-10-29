#!/bin/bash

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash
source devel/setup.bash

rm -rf src/ballsbot_detection && \
rm -rf src/ballsbot_laser_ranging_sensor && \
rm -rf src/ballsbot_tca9548 && \
rm -rf src/ballsbot_magnetic_encoder && \
rm -rf src/ballsbot_imu && \
rm -rf src/ballsbot_wheel_odometry && \
rm -rf src/ballsbot_pose && \
cd src && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_detection && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_laser_ranging_sensor && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_tca9548 && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_magnetic_encoder && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_imu && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_wheel_odometry && \
ln -s /home/ballsbot/projects/ballsbot/ros_modules/ballsbot_pose && \
cd .. && \

catkin_make

