from ballsbot.ros_messages import get_ros_messages


class UPS:
    def __init__(self):
        self.messenger = get_ros_messages()

    def _get_data(self):
        data = self.messenger.get_message_data('ups')
        if data:
            return {
                'voltage': data.voltage,
                'capacity': data.capacity,
            }
        else:
            return {
                'voltage': None,
                'capacity': None,
            }

    def get_voltage(self):
        return self._get_data()['voltage']

    def get_capacity(self):
        return self._get_data()['capacity']
