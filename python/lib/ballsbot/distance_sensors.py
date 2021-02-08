import sys
import numpy as np

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/home/ballsbot/catkin_ws/devel/lib/python2.7/dist-packages')

import rospy
from ballsbot_laser_ranging_sensor.msg import LaserDistance
from ballsbot.config import LASER_SENSOR_FRONT_ENABLED, LASER_SENSOR_REAR_ENABLED, \
    LASER_SENSOR_FRONT_OFFSET, LASER_SENSOR_REAR_OFFSET
from ballsbot.utils import run_as_thread


def has_distance_sensors():
    return LASER_SENSOR_FRONT_ENABLED or LASER_SENSOR_REAR_ENABLED


class DistanceSensors:
    def __init__(self, autostart=True):
        self.distances = {}
        self.avg_points_count = 4
        if autostart:
            self.start()

    def start(self):
        if LASER_SENSOR_FRONT_ENABLED:
            self.distances['front'] = {
                'shift': LASER_SENSOR_FRONT_OFFSET,
            }
            run_as_thread(self.get_auto_update('front'))

        if LASER_SENSOR_REAR_ENABLED:
            self.distances['rear'] = {
                'shift': LASER_SENSOR_REAR_OFFSET,
            }
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
                    it = self.distances[data.direction]
                    if 'values' in it:
                        it['values'][it['index']] = data.distance_in_mm
                        it['index'] = (it['index'] + 1) % self.avg_points_count
                    else:
                        it['values'] = [data.distance_in_mm] * self.avg_points_count
                        it['index'] = 0

        return result

    def get_distances(self):
        result = {}
        for k, it in self.distances.items():
            if 'values' not in it:
                result[k] = None
            else:
                value = np.mean(it['values'])
                if value > 1900.:
                    result[k] = None
                else:
                    result[k] = value + self.distances[k]['shift']
                    if result[k] < 0:
                        result[k] = 0.
        return result
