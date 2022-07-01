import logging
import json
from time import time

from ballsbot.config import MANIPULATOR
from ballsbot.manipulator import Manipulator
from ballsbot.manipulator_poses import Poses
from ballsbot.joystick import JoystickWrapperBase
from ballsbot.utils import keep_rps
from ballsbot.servos import float_map_range

logger = logging.getLogger(__name__)


class ManipulatorRunner:
    STAGE_START = 'start'
    STAGE_WAIT_INTENDED = 'wait_intended'
    STAGE_WAIT_REAL = 'wait_real'
    INTENDED_EPS = 0.05
    REAL_EPS = 0.2

    def __init__(self, poses_generator, joystick=None, manipulator=None, already_running=False):
        self.already_running = already_running
        self.joystick = (joystick or JoystickWrapperBase())
        self.manipulator = (manipulator or Manipulator(self.joystick))
        self.running = False
        self.poses_generator = poses_generator
        self.need_pose = None
        self.action_stage = None
        self.angles = None
        self.raw_pose = None

    def stop(self):
        self.running = False
        if not self.already_running:
            self.manipulator.stop()

    def _next_pose(self):
        self.raw_pose = self.poses_generator.get_next_pose()
        logging.info('moving to ' + ', '.join(str(x) for x in self.raw_pose))  # pylint: disable=W1201
        self.need_pose = self._convert_pose_to_angles(self.raw_pose)

    def run(self, save_track_to=None):
        if save_track_to:
            track_fh = open(save_track_to, 'w')  # pylint: disable=R1732
        else:
            track_fh = None

        self.running = True
        self.angles = self.manipulator.get_angles()
        self._next_pose()
        self.action_stage = self.STAGE_START
        ts = None
        while self.running:
            ts = keep_rps(ts, fps=4)

            self.angles = self.manipulator.get_angles()
            if self.action_stage == self.STAGE_START:
                self._set_intended_directions(self.need_pose)
                self.action_stage = self.STAGE_WAIT_INTENDED
            elif self.action_stage == self.STAGE_WAIT_INTENDED:
                if self._reached_intended_pose(self.need_pose):
                    self._set_no_direction()
                    self.action_stage = self.STAGE_WAIT_REAL
                else:
                    self._set_intended_directions(self.need_pose)
            elif self.action_stage == self.STAGE_WAIT_REAL:
                if self._reached_real_pose(self.need_pose):
                    self.action_stage = self.STAGE_START
                    self._next_pose()

            if track_fh is not None:
                self._store_track_frame(track_fh)

        if track_fh is not None:
            track_fh.close()

    def _convert_pose_to_angles(self, pose):
        result = []
        for servo_index, need_position in enumerate(pose):
            if need_position is None:
                need_angle = None
            else:
                servo_config = MANIPULATOR['servos'][servo_index]
                if isinstance(need_position, str):
                    if need_position == Poses.DEFAULT:
                        need_angle = float_map_range(
                            servo_config['default_position'],
                            -1., 1.,
                            servo_config['min_angle'], servo_config['max_angle']
                        )
                    elif need_position == Poses.UNFOLD:
                        need_angle = float_map_range(
                            servo_config.get('unfold_position', servo_config['default_position']),
                            -1., 1.,
                            servo_config['min_angle'], servo_config['max_angle']
                        )
                    elif need_position == Poses.MIN:
                        need_angle = servo_config['min_angle']
                    elif need_position == Poses.MAX:
                        need_angle = servo_config['max_angle']
                    else:
                        raise NotImplementedError(need_position)
                elif isinstance(need_position, dict):
                    increment = need_position['increment']
                    current_angle = self.angles[servo_index]['real']
                    if current_angle is None:
                        current_angle = self.angles[servo_index]['intended']
                        # TODO correct with lag?
                    need_angle = current_angle + increment
                    if servo_config['min_angle'] < servo_config['max_angle']:
                        min_angle = servo_config['min_angle']
                        max_angle = servo_config['max_angle']
                    else:
                        min_angle = servo_config['max_angle']
                        max_angle = servo_config['min_angle']
                    if need_angle < min_angle:
                        need_angle = min_angle
                    elif need_angle > max_angle:
                        need_angle = max_angle
                else:
                    if (
                            servo_config['min_angle'] < servo_config['max_angle'] and
                            servo_config['min_angle'] <= need_position <= servo_config['max_angle'] or
                            servo_config['min_angle'] > servo_config['max_angle'] and
                            servo_config['min_angle'] >= need_position >= servo_config['max_angle']
                    ):
                        need_angle = need_position
                    else:
                        raise OverflowError(
                            f'servo {servo_index}: {need_position} must be between '
                            f'{servo_config["min_angle"]} and {servo_config["max_angle"]}'
                        )
            result.append(need_angle)
        return result

    def _set_no_direction(self):
        self.joystick.manipulator_fold_pressed = 0
        self.joystick.manipulator_unfold_pressed = 0
        self.joystick.manipulator_directions = [0 for _ in self.joystick.manipulator_directions]

    def _set_intended_directions(self, pose):
        for servo_index, need_angle in enumerate(pose):
            if need_angle is None:
                continue
            current_angle_item = self.angles[servo_index]['intended']
            servo_config = MANIPULATOR['servos'][servo_index]
            if abs(current_angle_item - need_angle) <= self.INTENDED_EPS:
                self.joystick.manipulator_directions[servo_index] = 0
            elif servo_config['min_angle'] < servo_config['max_angle']:
                if current_angle_item < need_angle:
                    self.joystick.manipulator_directions[servo_index] = 1
                else:
                    self.joystick.manipulator_directions[servo_index] = -1
            else:
                if current_angle_item < need_angle:
                    self.joystick.manipulator_directions[servo_index] = -1
                else:
                    self.joystick.manipulator_directions[servo_index] = 1

    def _reached_intended_pose(self, pose):
        for servo_index, need_angle in enumerate(pose):
            if need_angle is None:
                continue
            if abs(self.angles[servo_index]['intended'] - need_angle) > self.INTENDED_EPS:
                return False
        return True

    def _reached_real_pose(self, pose):
        for servo_index, need_angle in enumerate(pose):
            if need_angle is None:
                continue
            current_angle = self.angles[servo_index]['real']
            if current_angle is None:
                current_angle = self.angles[servo_index]['intended']
            if abs(current_angle - need_angle) > self.REAL_EPS:
                return False
        return True

    def _store_track_frame(self, track_fh):
        frame = {
            'ts': time(),
            'need_pose': self.need_pose,
            'action_stage': self.action_stage,
            'manipulator': self.manipulator.get_track_frame(),
            'joystick': {
                'manipulator_fold_pressed': self.joystick.manipulator_fold_pressed,
                'manipulator_unfold_pressed': self.joystick.manipulator_unfold_pressed,
                'manipulator_directions': self.joystick.manipulator_directions,
            },
            'poses_generator': self.poses_generator.get_track_frame(),
        }
        json.dump(frame, track_fh)
        track_fh.write('\n')
        track_fh.flush()
