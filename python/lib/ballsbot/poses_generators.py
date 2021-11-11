from math import pi
import logging

from ballsbot.config import MANIPULATOR_DETECTION_MAX_DISTANCE_FROM_CENTER_X, \
    MANIPULATOR_DETECTION_MAX_DISTANCE_FROM_CENTER_Y
from ballsbot.detection import DetectorWrapper

logger = logging.getLogger(__name__)


class PosesCycle:
    def __init__(self, poses):
        self.poses = poses
        self.pose_index = 0

    def get_next_pose(self):
        result = self.poses[self.pose_index]
        self.pose_index = (self.pose_index + 1) % len(self.poses)
        return result

    def get_track_frame(self):
        return {
            'next_pose': self.poses[self.pose_index],
            'pose_index': self.pose_index,
        }


class DetectorPoses:
    STEP = 0.25  # radians

    def __init__(self):
        self.cycle = PosesCycle(self._get_guard_route_poses())
        self.detector_wrapper = DetectorWrapper(
            MANIPULATOR_DETECTION_MAX_DISTANCE_FROM_CENTER_X,
            MANIPULATOR_DETECTION_MAX_DISTANCE_FROM_CENTER_Y
        )

    def _get_guard_route_poses(self):
        poses_list = [
            (0., 0., 0., 0.),
        ]
        sectors = 5
        limit = pi / 3.
        claw_move = {'increment': -self.STEP}
        for i in range(1, sectors + 1):
            poses_list.append(
                (i * limit / sectors, None, None, claw_move),
            )
        for i in range(1, sectors + 1):
            poses_list.append(
                ((sectors - i) * limit / sectors, None, None, claw_move),
            )
        for i in range(1, sectors + 1):
            poses_list.append(
                (-i * limit / sectors, None, None, claw_move),
            )
        for i in range(1, sectors + 1):
            poses_list.append(
                (-(sectors - i) * limit / sectors, None, None, claw_move),
            )
        return poses_list

    def get_next_pose(self):
        self.detector_wrapper.update_detection()
        if self.detector_wrapper.cached_detected_object is None:
            return self.cycle.get_next_pose()
        else:
            segment_x, segment_y = self.detector_wrapper.get_detection_segment()

            if segment_x < 0:
                increment_x = {'increment': self.STEP}
            elif segment_x > 0:
                increment_x = {'increment': -self.STEP}
            else:
                increment_x = None

            if segment_y < 0:
                increment_y = {'increment': self.STEP}
            elif segment_y > 0:
                increment_y = {'increment': -self.STEP}
            else:
                increment_y = None

            return increment_x, 0., increment_y, {'increment': self.STEP}

    def get_track_frame(self):
        return {
            'cycle': self.cycle.get_track_frame(),
            'detector': self.detector_wrapper.get_track_frame(),
        }
