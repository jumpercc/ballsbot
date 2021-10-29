from copy import deepcopy

from ballsbot.ros_messages import get_ros_messages
from ballsbot.config import DISTANCE_SENSORS


def has_distance_sensors():
    return len(DISTANCE_SENSORS.keys())


class DistanceSensors:
    def __init__(self):
        self.default_distances = {}
        self.messenger = get_ros_messages()

    def start(self):
        if has_distance_sensors():
            for sensor_name, sensor_config in DISTANCE_SENSORS.items():
                self.default_distances[sensor_name] = {
                    'offset_x': sensor_config['offset_x'] / 1000.,
                    'offset_y': sensor_config.get('offset_y', 0.) / 1000.,
                    'direction': sensor_config['direction'],
                    'value': None,
                    'ts': None,
                }

    def get_distances(self):
        data_by_name = self.messenger.get_message_data('laser_distance')
        result = deepcopy(self.default_distances)
        if data_by_name:
            for sensor_name, data in data_by_name.items():
                result[sensor_name]['value'] = data.distance_in_mm / 1000.
                result[sensor_name]['ts'] = data.header.stamp.to_sec()

        for k in self.default_distances:
            if result[k]['value'] is None:
                result[k] = None
            else:
                value = result[k]['value']
                if value >= 2.5:
                    result[k] = None
                else:
                    result[k]['distance'] = value + result[k]['offset_x']
                    if result[k]['distance'] < 0:
                        result[k]['distance'] = 0.
                    del result[k]['value']
                    del result[k]['offset_x']
        return result
