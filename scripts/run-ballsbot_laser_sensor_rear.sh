#!/bin/bash

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash
source devel/setup.bash

rosrun ballsbot_laser_ranging_sensor publisher_laser_sensor 1 rear __name:=ballsbot_laser_ranging_sensor_rear

