import json
import sys

sys.path.append('/home/jumper/projects/ballsbot/lib')

from ballsbot.ai.explorer_mock import get_mocked_bot, TrackFeedBase


class StopException(Exception):
    pass


class TrackFeed(TrackFeedBase):
    def __init__(self, file_path):
        with open(file_path, 'r') as hf:
            self.track_info = json.loads(hf.read())
        self.frame_number = 0

    def get_current_frame(self):
        return self.track_info[self.frame_number]

    def next_frame(self):
        self.frame_number += 1
        if self.frame_number == len(self.track_info):
            raise StopException()


for _ in range(3):
    feed = TrackFeed('../track_info_01.json')
    bot = get_mocked_bot(feed)

    try:
        bot.run()
    except StopException:
        pass
