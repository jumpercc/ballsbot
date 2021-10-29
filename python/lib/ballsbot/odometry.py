from ballsbot.ros_messages import get_ros_messages


class Odometry:
    def __init__(self):
        self.messenger = get_ros_messages()

    def _get_data(self):
        data = self.messenger.get_message_data('wheel_odometry')
        if data:
            return {
                'odometry_counter': data.odometry_counter,
                'direction': data.direction,
                'speed': data.speed,
                'ts': data.header.stamp.to_sec(),
            }
        else:
            return {
                'odometry_counter': 0,
                'direction': 0.,
                'speed': 0.,
                'ts': None,
            }

    def get_speed(self):
        return self._get_data()['speed']

    def get_direction(self):
        return self._get_data()['direction']

    def get_ts(self):
        return self._get_data()['ts']

    def get_odometry_counter(self):
        return self._get_data()['odometry_counter']
