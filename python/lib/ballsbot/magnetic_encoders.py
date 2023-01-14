from copy import deepcopy
from math import pi

from ballsbot.ros_messages import get_ros_messages
from ballsbot.config import MANIPULATOR


def has_magnetic_encoders():
    return MANIPULATOR['enabled'] and MANIPULATOR.get('encoders_enabled')


class MagneticEncoders:
    def __init__(self):
        self.default_angles = {}
        self.messenger = get_ros_messages()
        self.servo_config_by_name = {}

    def start(self):
        if has_magnetic_encoders():
            for i, servo_config in enumerate(MANIPULATOR['servos']):
                sensor_name = servo_config['encoder_name']
                self.servo_config_by_name[sensor_name] = servo_config.copy()
                self.servo_config_by_name[sensor_name]['encoder_offset'] = MANIPULATOR['encoders_calibration'][i]
                self.default_angles[sensor_name] = {
                    'value': None,
                    'ts': None,
                    'raw_value': None,
                }

    def get_angles(self):
        data_by_name = self.messenger.get_message_data('magnetic_encoder')
        result = deepcopy(self.default_angles)
        if data_by_name:
            for sensor_name, data in data_by_name.items():
                result[sensor_name]['raw_value'] = data.angle
                servo_config = self.servo_config_by_name[sensor_name]
                reverse = -1. if servo_config.get('reverse_encoder') else 1.
                result[sensor_name]['value'] = reverse * (data.angle - servo_config['encoder_offset'])
                if result[sensor_name]['value'] < -pi:
                    result[sensor_name]['value'] += 2 * pi
                elif result[sensor_name]['value'] > pi:
                    result[sensor_name]['value'] -= 2 * pi
                result[sensor_name]['ts'] = data.header.stamp.to_sec()
        return result
