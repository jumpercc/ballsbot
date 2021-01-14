import RTIMU
from time import time, sleep
import numpy as np
from math import pi
from ballsbot.utils import run_as_thread

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

    def main_loop(self):
        while True:
            if self.imu.IMURead():
                data = self.imu.getIMUData()
                if data["fusionPose"][0] != 0. or data["fusionPose"][1] != 0. or data["fusionPose"][2] != 0.:
                    diff = np.array(data["fusionPose"]) - self.initial_position
                    teta = -diff[2]
                    if teta > pi:
                        teta -= 2 * pi
                    self.teta = teta
                    self.teta_ts = time()
                    self.w_z = data['gyro'][2]  # FIXME move axis via quaternions at al
                sleep(self.poll_interval * 1.0 / 1000.0)

    def start(self):
        self.calibrate()
        self.main_loop()


class IMU_Threaded:
    def __init__(self):
        run_as_thread(self.start)

    def start(self):
        self.imu = IMU()
        self.imu.start()

    def get_teta(self):
        return self.imu.teta

    def get_w_z(self):
        return self.imu.w_z

    def get_teta_ts(self):
        return self.imu.teta_ts
