import argparse
import sys
import logging

logging.basicConfig(
    format='[%(asctime)s] %(levelname)s %(name)s: %(message).700s',
    stream=sys.stderr,
    level=logging.INFO
)
logger = logging.getLogger(__name__)

sys.path.append('/home/ballsbot/projects/ballsbot/python/lib')

from ballsbot.manipulator_runner import ManipulatorRunner
from ballsbot.manipulator_poses import Poses
from ballsbot.poses_generators import PosesCycle
from ballsbot.utils import run_as_thread, join_all_threads
from ballsbot.ros_messages import get_ros_messages


def main():
    parser = argparse.ArgumentParser(description='Start ballsbot manipulator loop.')
    parser.add_argument('--save-track-to', dest='save_track_to', help='save track filepath')
    args = parser.parse_args()

    poses_generator = PosesCycle([
        (Poses.DEFAULT, Poses.DEFAULT, Poses.DEFAULT, Poses.DEFAULT),
        (0., 0., 0., 0.),
        (Poses.MAX, None, None, Poses.MIN),
        (Poses.MIN, None, None, Poses.MAX),
        (0., Poses.MAX, Poses.MIN, Poses.MIN),
        (0., 0., 0., None),
    ])
    bot = ManipulatorRunner(poses_generator)
    run_as_thread(lambda: bot.run(save_track_to=args.save_track_to))
    get_ros_messages().start(sync=True)
    bot.stop()
    join_all_threads()


if __name__ == '__main__':
    main()
