from time import sleep
import sys
import logging
import json
from math import pi

logging.basicConfig(
    format='[%(asctime)s] %(levelname)s %(name)s: %(message).700s',
    stream=sys.stderr,
    level=logging.INFO
)
logger = logging.getLogger(__name__)

sys.path.append('/home/ballsbot/projects/ballsbot/python/lib')

from ballsbot.config import MANIPULATOR
from ballsbot.manipulator_runner import ManipulatorRunner
from ballsbot.manipulator_poses import Poses
from ballsbot.utils import run_as_thread, join_all_threads
from ballsbot.magnetic_encoders import MagneticEncoders, has_magnetic_encoders
from ballsbot.ros_messages import get_ros_messages

POSES = [
    (Poses.DEFAULT, Poses.MIN, Poses.MAX, Poses.MIN),
    (Poses.MAX, Poses.MAX, Poses.MIN, Poses.MAX),
    (Poses.MIN, Poses.MAX, Poses.MIN, Poses.MAX),
]


class PosesForCalibration:
    def __init__(self):
        self.poses = [
            (Poses.DEFAULT, Poses.DEFAULT, Poses.DEFAULT, Poses.DEFAULT),
        ]
        self.pose_index = 0

    def get_next_pose(self):
        result = self.poses[self.pose_index]
        return result

    def get_track_frame(self):
        return {
            'next_pose': self.poses[self.pose_index],
            'pose_index': self.pose_index,
        }

    def set_pose(self, pose):
        self.poses.append(pose)
        self.pose_index += 1


def extract_raw_angles(data):
    result = {}
    for key, it in data.items():
        result[key] = it['raw_value']
    return result


def angle_to_range(angle):
    if angle < -pi:
        angle += 2 * pi
    elif angle > pi:
        angle -= 2 * pi
    return angle


def start_overlord(poses_generator, encoders, bot):
    result = {}
    sleep(10)

    for pose in POSES:
        poses_generator.set_pose(pose)
        if not bot.running:
            return
        sleep(20)
        logger.info('saving angles')
        result[pose] = extract_raw_angles(encoders.get_angles())

    poses_generator.set_pose((Poses.DEFAULT, Poses.DEFAULT, Poses.DEFAULT, Poses.DEFAULT))
    if not bot.running:
        return
    sleep(10)

    offsets = []
    patches = {}
    for i, it in enumerate(MANIPULATOR['servos']):
        if i == 0:
            offset = result[POSES[0]][it['encoder_name']]
            min_angle = result[POSES[2]][it['encoder_name']] - offset
            max_angle = result[POSES[1]][it['encoder_name']] - offset
            patches[it['encoder_name']] = {
                'min_angle': round(angle_to_range(min_angle), 4),
                'max_angle': round(angle_to_range(max_angle), 4),
            }
        else:
            reverse = -1. if it.get('reverse_encoder') else 1.
            known_value = it[f'{POSES[0][i]}_angle'] * reverse
            raw_value = result[POSES[0]][it['encoder_name']]
            offset = raw_value - known_value
            other_end_name = f'{POSES[1][i]}_angle'
            other_end_value = reverse * (result[POSES[1]][it['encoder_name']] - offset)
            patches[it['encoder_name']] = {
                other_end_name: round(angle_to_range(other_end_value), 4),
            }
        offsets.append(round(offset, 4))
    logger.info(json.dumps(offsets, indent=4))
    logger.info(json.dumps(patches, indent=4, sort_keys=True))


def main():
    assert has_magnetic_encoders()

    poses_generator = PosesForCalibration()
    bot = ManipulatorRunner(poses_generator, without_encoders=True)
    run_as_thread(lambda: bot.run(save_track_to=None))

    encoders = MagneticEncoders()
    encoders.start()
    run_as_thread(lambda: start_overlord(poses_generator, encoders, bot))

    get_ros_messages().start(sync=True)
    bot.stop()
    join_all_threads()


if __name__ == '__main__':
    main()
