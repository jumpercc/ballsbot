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
from ballsbot.poses_generators import DetectorPoses
from ballsbot.utils import run_as_thread, join_all_threads
from ballsbot.ros_messages import get_ros_messages


def main():
    parser = argparse.ArgumentParser(description='Start ballsbot manipulator detector.')
    parser.add_argument('--save-track-to', dest='save_track_to', help='save track filepath')
    args = parser.parse_args()

    poses_generator = DetectorPoses()
    bot = ManipulatorRunner(poses_generator)  # TODO add callback for logging
    run_as_thread(lambda: bot.run(save_track_to=args.save_track_to))
    get_ros_messages().start(sync=True)
    bot.stop()
    join_all_threads()


if __name__ == '__main__':
    main()
