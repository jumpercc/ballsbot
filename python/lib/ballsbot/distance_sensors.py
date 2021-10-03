import sys
import numpy as np

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/home/ballsbot/catkin_ws/devel/lib/python2.7/dist-packages')

import rospy
from ballsbot_laser_ranging_sensor.msg import LaserDistance as LaserDistanceStraight
from ballsbot_tca9548.msg import LaserDistance as LaserDistanceSwitch
from ballsbot.config import DISTANCE_SENSORS, DISTANCE_SENSORS_MESSAGE_TYPE
from ballsbot.utils import run_as_thread

CONFIG_BY_TYPE = {
    "ballsbot_laser_ranging_sensor": {
        "class": LaserDistanceStraight,
        "get_topic": lambda sensor_name: '/laser_distance_' + sensor_name,
    },
    "ballsbot_tca9548": {
        "class": LaserDistanceSwitch,
        "get_topic": lambda _: '/laser_distance',
    },
}


def has_distance_sensors():
    return len(DISTANCE_SENSORS.keys())


class DistanceSensors:
    def __init__(self, autostart=True):
        self.distances = {}
        self.avg_points_count = 4
        if autostart:
            self.start()

    def start(self):
        if has_distance_sensors():
            for sensor_name, sensor_config in DISTANCE_SENSORS.items():
                self.distances[sensor_name] = {
                    'shift': sensor_config['offset'],
                }
                if DISTANCE_SENSORS_MESSAGE_TYPE == "ballsbot_laser_ranging_sensor":
                    run_as_thread(self.get_auto_update(sensor_name))
            if DISTANCE_SENSORS_MESSAGE_TYPE != "ballsbot_laser_ranging_sensor":
                run_as_thread(self.get_auto_update(None))

    def get_auto_update(self, sensor_name):
        message_class = CONFIG_BY_TYPE[DISTANCE_SENSORS_MESSAGE_TYPE]["class"]
        topic = CONFIG_BY_TYPE[DISTANCE_SENSORS_MESSAGE_TYPE]["get_topic"](sensor_name)

        def result():
            while True:
                try:
                    data = rospy.wait_for_message(topic, message_class, timeout=5)
                except KeyboardInterrupt:
                    break
                except rospy.exceptions.ROSException:
                    data = None
                if data is not None:
                    it = self.distances[data.sensor_name]
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

    @classmethod
    def get_directions(cls):
        return {k: v['direction'] for k, v in DISTANCE_SENSORS.items()}
