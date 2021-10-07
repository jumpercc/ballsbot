#!/usr/bin/env python3
import RTIMU
from time import time, sleep
import numpy as np
from math import pi
import sys
import os
import pwd

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/home/ballsbot/catkin_ws/devel/lib/python2.7/dist-packages')

import rospy
from ballsbot_imu.msg import ImuState


def get_username():
    return pwd.getpwuid(os.getuid())[0]


s = RTIMU.Settings("RTIMULib")


class IMU:
    def __init__(self):
        imu = RTIMU.RTIMU(s)
        if not imu.IMUInit():
            raise ValueError("IMU Init Failed")

        imu.setSlerpPower(0.02)
        imu.setGyroEnable(True)
        imu.setAccelEnable(True)
        imu.setCompassEnable(True)

        self.imu = imu
        self.poll_interval = imu.IMUGetPollInterval()
        self.initial_position = np.zeros((3,))
        self.teta = 0.
        self.teta_ts = time()
        self.w_z = 0.

    def calibrate(self):
        iterations = 0
        my_sum = np.zeros((3,))
        while True:
            if self.imu.IMURead():
                data = self.imu.getIMUData()
                if data["fusionPose"][0] != 0. or data["fusionPose"][1] != 0. or data["fusionPose"][2] != 0.:
                    my_sum += np.array(data["fusionPose"])
                    iterations += 1
                    if iterations == 10:
                        break
            sleep(self.poll_interval * 1.0 / 1000.0)
        self.initial_position = my_sum / iterations

    def get_teta(self):
        return self.teta

    def get_w_z(self):
        return self.w_z

    def get_teta_ts(self):
        return self.teta_ts


def publisher():
    imu = IMU()
    imu.calibrate()

    pub = rospy.Publisher('imu', ImuState, queue_size=2)
    rospy.init_node('ballsbot_imu')
    rate = rospy.Rate(10)
    zero_duration = rospy.rostime.Duration(0, 0)
    while not rospy.is_shutdown():
        if imu.imu.IMURead():
            data = imu.imu.getIMUData()
            if data["fusionPose"][0] != 0. or data["fusionPose"][1] != 0. or data["fusionPose"][2] != 0.:
                diff = np.array(data["fusionPose"]) - imu.initial_position
                teta = -diff[2]
                if teta > pi:
                    teta -= 2 * pi
                imu.teta = teta
                imu.teta_ts = time()
                imu.w_z = data['gyro'][2]  # FIXME move axis via quaternions at al

        if rate.remaining() <= zero_duration:
            rate.last_time = rospy.rostime.get_rostime()
            message = ImuState()
            message.teta = imu.get_teta()
            message.w_z = imu.get_w_z()
            message.teta_ts = imu.get_teta_ts()
            rospy.loginfo(message.teta)
            pub.publish(message)

        rospy.sleep(imu.poll_interval * 1.0 / 1000.0)


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
