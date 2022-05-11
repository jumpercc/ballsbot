#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ -z ${NO_MAKE_ROS_MODULES+x} ]; then
    $DIR/make-ros-modules.sh
    if [ $? -ne 0 ]
    then
      exit 2
    fi
fi

$DIR/run-ballsbot_laser_sensors_change_addresses.sh
if [ $? -ne 0 ]
then
  exit 3
fi

$DIR/run-roscore.sh > ~/core.log 2>&1 &

sleep 5s  # wait for roscore to run TODO replace with roslaunch

$DIR/run-lidar.sh > ~/lidar.log 2>&1 &
sleep 1

$DIR/run-ballsbot_laser_sensor_front.sh > ~/sensor_front.log 2>&1 &
sleep 1

$DIR/run-ballsbot_laser_sensor_rear.sh > ~/sensor_rear.log 2>&1 &
sleep 1

$DIR/run-ballsbot_imu.sh > ~/imu.log 2>&1 &
sleep 1

$DIR/run-ballsbot_wheel_odometry.sh > ~/wheel_odometry.log 2>&1 &
sleep 1

$DIR/run-ballsbot_pose.sh > ~/pose.log 2>&1 &
sleep 1

if [ -z ${NO_DETECTION+x} ]; then
    $DIR/run-ballsbot_detection.sh > ~/detection.log 2>&1 &
    sleep 1
fi

if [ -z ${LOCAL_PORT+x} ]; then
  MY_IP=$(ifconfig | grep -P '\binet\b' | grep -vF 'inet 127.0.0.1' | head -1 | awk '{print $2}')
else
  MY_IP=127.0.0.1
fi

if [ -z ${TELEOPERATION+x} ]; then
  $DIR/run-jupyter.sh $MY_IP
else
  $DIR/run-teleoperation.sh $MY_IP
fi

kill %2 2>/dev/null
kill %3 2>/dev/null
kill %4 2>/dev/null
kill %5 2>/dev/null
kill %6 2>/dev/null
kill %7 2>/dev/null

kill %1 2>/dev/null
