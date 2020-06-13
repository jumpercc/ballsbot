import RTIMU
import time
import json
import numpy as np
from math import pi
import atexit
import threading


class IMU:
    def __init__(self):
        s = RTIMU.Settings("RTIMULib")
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
            time.sleep(self.poll_interval * 1.0 / 1000.0)
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
                time.sleep(self.poll_interval * 1.0 / 1000.0)

    def start(self):
        self.calibrate()
        self.main_loop()


class IMU_Threaded:
    def __init__(self):
        self.thread = threading.Thread(target=self.start)
        self.thread.start()
        atexit.register(self.stop)

    def start(self):
        self.imu = IMU()
        self.imu.start()

    def stop(self):
        self.thread.join()

    def get_teta(self):
        return self.imu.teta
