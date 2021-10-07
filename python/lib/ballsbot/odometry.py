from ballsbot.utils import run_as_thread
import ballsbot.session  # pylint: disable=W0611

import rospy
from ballsbot_wheel_odometry.msg import OdometryState


class Odometry:
    def __init__(self):
        self.odometry_counter = 0
        self.direction = 0.
        self.speed = 0.
        run_as_thread(self.start)

    def start(self):
        while True:
            try:
                data = rospy.wait_for_message('/wheel_odometry', OdometryState, timeout=5)
                self.odometry_counter = data.odometry_counter
                self.direction = data.direction
                self.speed = data.speed
            except KeyboardInterrupt:
                return None
            except Exception:  # pylint: disable=W0703
                pass

    def get_speed(self):
        return self.speed

    def get_direction(self):
        return self.direction
