import ballsbot.session  # pylint: disable=W0611

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
        if autostart:
            self.start()

    def start(self):
        if has_distance_sensors():
            for sensor_name, sensor_config in DISTANCE_SENSORS.items():
                self.distances[sensor_name] = {
                    'offset_x': sensor_config['offset_x'],
                    'offset_y': sensor_config.get('offset_y', 0),
                    'direction': sensor_config['direction'],
                    'value': None,
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
                    self.distances[data.sensor_name]['value'] = data.distance_in_mm

        return result

    def get_distances(self):
        result = {}
        for k, it in self.distances.items():
            if it['value'] is None:
                result[k] = None
            else:
                value = it['value']
                if value >= 2500.:
                    result[k] = None
                else:
                    result[k] = {
                        'distance': value + self.distances[k]['offset_x'],
                        'direction': it['direction'],
                        'offset_y': it['offset_y'],
                    }
                    if result[k]['distance'] < 0:
                        result[k]['distance'] = 0.
        return result
