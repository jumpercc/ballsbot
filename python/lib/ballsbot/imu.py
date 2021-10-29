from ballsbot.ros_messages import get_ros_messages


class IMU:
    def __init__(self):
        self.messenger = get_ros_messages()

    def _get_data(self):
        data = self.messenger.get_message_data('imu')
        if data:
            return {
                'teta': data.teta,
                'ts': data.header.stamp.to_sec(),
                'w_z': data.w_z,
            }
        else:
            return {
                'teta': 0.,
                'ts': None,
                'w_z': 0.,
            }

    def get_teta(self):
        return self._get_data()['teta']

    def get_w_z(self):
        return self._get_data()['w_z']

    def get_teta_ts(self):
        return self._get_data()['ts']
