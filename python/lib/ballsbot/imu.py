from time import time
from ballsbot.utils import run_as_thread
import ballsbot.session  # pylint: disable=W0611

import rospy
from ballsbot_imu.msg import ImuState


class IMU:
    def __init__(self):
        self.teta = 0.
        self.teta_ts = time()
        self.w_z = 0.
        run_as_thread(self.start)

    def start(self):
        while True:
            try:
                data = rospy.wait_for_message('/imu', ImuState, timeout=5)
                self.teta = data.teta
                self.teta_ts = data.teta_ts
                self.w_z = data.w_z
            except KeyboardInterrupt:
                return None
            except Exception:  # pylint: disable=W0703
                pass

    def get_teta(self):
        return self.teta

    def get_w_z(self):
        return self.w_z

    def get_teta_ts(self):
        return self.teta_ts
