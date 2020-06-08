import RTIMU
import time
from IPython.display import clear_output
import json


class IMU:
    def __init__(self):
        s = RTIMU.Settings("RTIMULib")
        imu = RTIMU.RTIMU(s)
        if not imu.IMUInit():
            raise ValueError("IMU Init Failed")

        # this is a good time to set any fusion parameters
        imu.setSlerpPower(0.02)
        imu.setGyroEnable(True)
        imu.setAccelEnable(True)
        # imu.setCompassEnable(True)

        self.imu = imu
        self.poll_interval = imu.IMUGetPollInterval()

    @staticmethod
    def extract_values(data):
        return {
            "pose": data["fusionPose"],
            "gyro": data["gyro"],
            "accel": data["accel"],
        }

    def main_loop(self):
        while True:
            clear_output(wait=True)
            if self.imu.IMURead():
                data = self.imu.getIMUData()
                print(json.dumps(self.extract_values(data), indent=4))
                time.sleep(self.poll_interval * 1.0 / 1000.0)
