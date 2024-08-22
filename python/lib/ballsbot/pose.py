from ballsbot.ros_messages import get_ros_messages


class Pose:
    def __init__(self):
        self.messenger = get_ros_messages()

    def _get_data(self):
        data = self.messenger.get_message_data('pose')
        if data:
            current_pose = {
                'raw_pose_ts': data.pose_ts.to_sec(),
                'self_ts': data.header.stamp.to_sec(),
                'x': data.x,
                'y': data.y,
                'teta': data.teta,
            }
            current_pose['ts'] = current_pose['raw_pose_ts']
        else:
            current_pose = {
                'raw_pose_ts': None,
                'self_ts': None,
                'x': None,
                'y': None,
                'teta': None,
                'ts': None,
            }
        return current_pose

    def get_pose(self):
        return self._get_data()
