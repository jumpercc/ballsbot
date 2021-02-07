import sys

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/home/ballsbot/catkin_ws/devel/lib/python2.7/dist-packages')

import rospy
from ballsbot_laser_ranging_sensor.msg import LaserDistance
from ballsbot.config import LASER_SENSOR_FRONT_ENABLED, LASER_SENSOR_REAR_ENABLED
from ballsbot.utils import run_as_thread


class DistanceSensors:
    def __init__(self):
        self.distances = {}

        if LASER_SENSOR_FRONT_ENABLED:
            self.distances['front'] = None
            run_as_thread(self.get_auto_update('front'))

        if LASER_SENSOR_REAR_ENABLED:
            self.distances['rear'] = None
            run_as_thread(self.get_auto_update('rear'))

    def get_auto_update(self, direction):
        def result():
            while True:
                try:
                    data = rospy.wait_for_message('/laser_distance_' + direction, LaserDistance, timeout=5)
                except KeyboardInterrupt:
                    break
                except rospy.exceptions.ROSException as e:
                    data = None
                if data is not None:
                    self.distances[data.direction] = data.distance_in_mm

        return result

    def get_distances(self):
        return self.distances.copy()
