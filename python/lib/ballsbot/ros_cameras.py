from ballsbot.ros_messages import get_ros_messages
from ballsbot.config import MANIPULATOR


class Cameras:
    def __init__(self):
        self.messenger = get_ros_messages()
        if MANIPULATOR.get('has_camera'):
            self.camera_indexes = [0, 1]
        else:
            self.camera_indexes = [0]

    def get_images(self):
        return {
            x: self.messenger.get_message_data(f'cam_image_{x}') for x in self.camera_indexes
        }
