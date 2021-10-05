import ballsbot.utils as ballsbot_utils

ballsbot_utils.keep_rps = lambda ts, fps: ts

from ballsbot.ai.explorer import Explorer
from ballsbot.lidar import Lidar, radial_points_to_cartesian


class TrackFeedBase:
    def get_current_frame(self):
        raise NotImplementedError()

    def next_frame(self):
        raise NotImplementedError()


class LidarMock:
    def __init__(self, feed):
        self.lidar = Lidar(True)
        self.calibration = None
        self.feed = feed

    def calibration_to_xywh(self, _):
        return self.lidar.calibration_to_xywh(self.lidar.calibration)

    def get_lidar_points(self):
        return radial_points_to_cartesian(self.get_radial_lidar_points(False))

    def get_radial_lidar_points(self, range_limit=None, cached=False):  # pylint: disable=W0613
        return self.feed.get_current_frame()['points']

    def radial_points_to_cartesian(self, points):  # pylint: disable=R0201
        return radial_points_to_cartesian(points)


class CarControlsMock:
    def __init__(self, feed):
        self.feed = feed
        self.counter = 0

    def run(self, _):
        self.counter += 1
        if self.counter == 2:
            self.counter = 0
            self.feed.next_frame()


class OdometryMock:
    def __init__(self, feed):
        self.feed = feed

    def get_speed(self):
        return self.feed.get_current_frame()['speed']

    def get_direction(self):
        return self.feed.get_current_frame()['direction']


class TrackerMock:
    def __init__(self, feed):
        self.feed = feed

    def get_current_pose(self):
        return self.feed.get_current_frame()['pose']


def get_mocked_bot(feed):
    car_controls = CarControlsMock(feed)
    return Explorer(profile_mocks={
        'lidar': LidarMock(feed),
        'car_controls': {'steering': car_controls, 'throttle': car_controls},
        'odometry': OdometryMock(feed),
        'tracker': TrackerMock(feed),
    })
