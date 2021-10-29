import argparse
import sys
import logging

logging.basicConfig(
    format='[%(asctime)s] %(levelname)s %(name)s: %(message).700s',
    stream=sys.stderr,
    level=logging.INFO
)

sys.path.append('/home/ballsbot/projects/ballsbot/python/lib')

from ballsbot.ai.explorer import Explorer
from ballsbot.utils import run_as_thread, join_all_theads
from ballsbot.ros_messages import get_ros_messages


def main():
    parser = argparse.ArgumentParser(description='Start ballsbot explorer.')
    parser.add_argument('--save-track-to', dest='save_track_to', help='save track filepath')
    args = parser.parse_args()

    bot = Explorer()
    run_as_thread(lambda: bot.run(save_track_to=args.save_track_to))
    get_ros_messages().start(sync=True)
    bot.stop()
    join_all_theads()


if __name__ == '__main__':
    main()
