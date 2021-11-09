import argparse
import sys
import logging
import json
from time import time

logging.basicConfig(
    format='[%(asctime)s] %(levelname)s %(name)s: %(message).700s',
    stream=sys.stderr,
    level=logging.INFO
)
logger = logging.getLogger(__name__)

sys.path.append('/home/ballsbot/projects/ballsbot/python/lib')

from ballsbot.config import MANIPULATOR
from ballsbot.manipulator import Manipulator
from ballsbot.joystick import JoystickWrapperBase
from ballsbot.utils import run_as_thread, join_all_theads, keep_rps
from ballsbot.servos import float_map_range
from ballsbot.ros_messages import get_ros_messages


class PosesCycle:
    def __init__(self, poses):
        self.poses = poses
        self.pose_index = 0

    def get_next_pose(self):
        result = self.poses[self.pose_index]
        self.pose_index = (self.pose_index + 1) % len(self.poses)
        return result


class ManipulatorRunner:
    STAGE_START = 'start'
    STAGE_WAIT_INTENDED = 'wait_intended'
    STAGE_WAIT_REAL = 'wait_real'
    POSITION_DEFAULT = 'default'
    POSITION_UNFOLD = 'unfold'
    POSITION_MIN = 'min'
    POSITION_MAX = 'max'
    INTENDED_EPS = 0.05
    REAL_EPS = 0.1

    def __init__(self, poses_generator):
        self.joystick = JoystickWrapperBase()
        self.manipulator = Manipulator(self.joystick)
        self.running = False
        self.poses_generator = poses_generator
        self.need_pose = None
        self.action_stage = None
        self.angles = None
        self.raw_pose = None

    def stop(self):
        self.running = False
        self.manipulator.stop()

    def _next_pose(self):
        self.raw_pose = self.poses_generator.get_next_pose()
        logging.info('moving to ' + ', '.join(str(x) for x in self.raw_pose))
        self.need_pose = self._convert_pose_to_angles(self.raw_pose)

    def run(self, save_track_to=None):
        if save_track_to:
            track_fh = open(save_track_to, 'w')  # pylint: disable=R1732
        else:
            track_fh = None

        self.running = True
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
                    if need_position == self.POSITION_DEFAULT:
                        need_angle = float_map_range(
                            servo_config['default_position'],
                            -1., 1.,
                            servo_config['min_angle'], servo_config['max_angle']
                        )
                    elif need_position == self.POSITION_UNFOLD:
                        need_angle = float_map_range(
                            servo_config.get('unfold_position', servo_config['default_position']),
                            -1., 1.,
                            servo_config['min_angle'], servo_config['max_angle']
                        )
                    elif need_position == self.POSITION_MIN:
                        need_angle = servo_config['min_angle']
                    elif need_position == self.POSITION_MAX:
                        need_angle = servo_config['max_angle']
                    else:
                        raise NotImplementedError(need_position)
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
            }
        }
        json.dump(frame, track_fh)
        track_fh.write('\n')
        track_fh.flush()


def main():
    parser = argparse.ArgumentParser(description='Start ballsbot explorer.')
    parser.add_argument('--save-track-to', dest='save_track_to', help='save track filepath')
    args = parser.parse_args()

    poses_generator = PosesCycle([
        (ManipulatorRunner.POSITION_DEFAULT, ManipulatorRunner.POSITION_DEFAULT, ManipulatorRunner.POSITION_DEFAULT,
         ManipulatorRunner.POSITION_DEFAULT),
        (0., 0., 0., 0.),
        (ManipulatorRunner.POSITION_MAX, None, None, ManipulatorRunner.POSITION_MIN),
        (ManipulatorRunner.POSITION_MIN, None, None, ManipulatorRunner.POSITION_MAX),
        (0., ManipulatorRunner.POSITION_MAX, ManipulatorRunner.POSITION_MIN, ManipulatorRunner.POSITION_MIN),
        (0., 0., 0., None),
    ])
    bot = ManipulatorRunner(poses_generator)
    run_as_thread(lambda: bot.run(save_track_to=args.save_track_to))
    get_ros_messages().start(sync=True)
    bot.stop()
    join_all_theads()


if __name__ == '__main__':
    main()
