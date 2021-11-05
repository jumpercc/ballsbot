#!/bin/bash

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash
source devel/setup.bash

rosrun ballsbot_tca9548 publisher_tca9548 0 119
