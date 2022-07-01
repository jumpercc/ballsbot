import json
from time import time
import logging
from ballsbot.lidar_with_memory import LidarWithMemory
from ballsbot.servos import get_controls
from ballsbot.utils import keep_rps
from ballsbot.tracking import Tracker, get_car_info
from ballsbot.detection import DetectorWrapper
from ballsbot.config import ENGINE_NEED_MANUAL_BREAKING, DETECTION_CLOSE_ENOUGH
from ballsbot_localization import ballsbot_localization as grid

logger = logging.getLogger(__name__)


class ExplorerDriverWithAutoBreaking:
    @staticmethod
    def fix_next_move(next_move, _):
        return next_move


class ExplorerDriverWithManualBreaking:
    STOP = 0.
    FORWARD_THROTTLE = 1.
    BACKWARD_THROTTLE = -1.
    FORWARD_BRAKE = -0.5
    BACKWARD_BRAKE = 0.95  # not a break just a reverse

    def fix_next_move(self, next_move, prev_move):
        # TODO detect jamming state and fix it
        if prev_move.get('steps_count', 0) > 0:
            move = prev_move.copy()
            move['steps_count'] -= 1
            return move
        elif prev_move['throttle'] == next_move['throttle'] or prev_move['throttle'] == self.STOP:
            return next_move
        elif prev_move['throttle'] == self.BACKWARD_BRAKE and next_move['throttle'] == self.FORWARD_THROTTLE:
            return next_move
        elif prev_move['throttle'] in {self.FORWARD_BRAKE, self.BACKWARD_BRAKE}:
            return {'steering': next_move['steering'], 'throttle': self.STOP}
        elif prev_move['throttle'] == self.FORWARD_THROTTLE:
            return {'steering': next_move['steering'], 'throttle': self.FORWARD_BRAKE, 'steps_count': 2}
        elif prev_move['throttle'] == self.BACKWARD_THROTTLE:
            return {'steering': next_move['steering'], 'throttle': self.BACKWARD_BRAKE}

        raise RuntimeError(f'invalid translation {prev_move} -> {next_move}')


class Explorer:
    FORWARD_THROTTLE = 1.
    BACKWARD_THROTTLE = -1.
    STOP = 0.

    def __init__(self, augmented_lidar=None, detector=None, joystick=None, already_running=False):
        if ENGINE_NEED_MANUAL_BREAKING:
            self.direction_translator = ExplorerDriverWithManualBreaking()
        else:
            self.direction_translator = ExplorerDriverWithAutoBreaking()

        self.lidar = LidarWithMemory(augmented_lidar=augmented_lidar, already_running=already_running)
        if joystick:
            self.joystick = joystick
            self.car_controls = None
        else:
            self.car_controls = get_controls()
            self.joystick = None
        self.tracker = Tracker(self.lidar)
        self.detector_wrapper = DetectorWrapper(detector=detector, already_running=already_running)

        self.cached_weights = None
        self.cached_direction = None
        self.running = False
        self.move_ts = None
        self.pose_lag = 0.

    def stop(self):
        self.running = False
        self.tracker.stop()

    def run(self, save_track_to=None):
        if save_track_to:
            track_fh = open(save_track_to, 'w')  # pylint: disable=R1732
        else:
            track_fh = None

        self.tracker.start()

        self.running = True
        self.cached_direction = {'steering': 0., 'throttle': 0.}
        ts = None
        while True:
            ts = keep_rps(ts, fps=2)
            running = self.running
            if running:
                self.cached_direction = self.direction_translator.fix_next_move(
                    self._get_next_move(self.cached_direction),
                    self.cached_direction
                )
            else:
                self.cached_direction = {'steering': 0., 'throttle': 0.}
            self._follow_direction(self.cached_direction)
            if track_fh is not None:
                self._store_track_frame(track_fh)
            if not running:
                break

        if track_fh is not None:
            track_fh.close()

    def _get_next_move(self, prev_direction):
        if not self.lidar.get_points_ts():
            return {'steering': 0., 'throttle': 0.}
        self.move_ts = time()
        self.pose_lag = self.move_ts - self.lidar.get_points_ts()
        if self.pose_lag >= 1.:
            logger.warning('pose_lag=%s, waiting', self.pose_lag)
            return {'steering': 0., 'throttle': 0.}
        weights = grid.get_directions_weights(
            self.lidar.get_points_ts(),
            get_car_info()
        )

        self.detector_wrapper.update_detection()
        if self.detector_wrapper.cached_detected_object is None:
            for sector_key in weights.keys():
                if sector_key[1] > 0. and prev_direction['throttle'] == self.FORWARD_THROTTLE \
                        or sector_key[1] < 0. and prev_direction['throttle'] == self.BACKWARD_THROTTLE:
                    weights[sector_key] *= 1.1  # trying to keep prev direction
                if sector_key[0] == prev_direction['steering']:
                    weights[sector_key] *= 1.05  # trying to keep wheel movements smooth
        else:
            directions = self._get_preferred_directions(self.detector_wrapper.cached_detected_object)
            if directions is None:
                for sector_key in weights.keys():
                    weights[sector_key] = 0.
            else:
                for sector_key in weights.keys():
                    if sector_key in directions:
                        weights[sector_key] *= 10.
                    else:
                        weights[sector_key] /= 10.

        self.cached_weights = weights
        weights = dict(filter(lambda x: x[1] > 0., weights.items()))
        if len(weights.keys()) > 0:
            sector_key = list(sorted(weights.items(), key=lambda x: x[1]))[-1][0]
        else:
            sector_key = (0., 0.)

        steering = sector_key[0]
        if sector_key[1] > 0.:
            throttle = self.FORWARD_THROTTLE
        elif sector_key[1] < 0.:
            throttle = self.BACKWARD_THROTTLE
        else:
            throttle = 0.

        if prev_direction['throttle'] == self.FORWARD_THROTTLE and throttle == self.BACKWARD_THROTTLE:
            throttle = self.STOP
        elif prev_direction['throttle'] == self.BACKWARD_THROTTLE and throttle == self.FORWARD_THROTTLE:
            throttle = self.STOP

        return {'steering': steering, 'throttle': throttle}

    def _follow_direction(self, direction):
        if self.joystick:
            self.joystick.steering = direction['steering']
            self.joystick.throttle = direction['throttle']
        else:
            self.car_controls['steering'].run(direction['steering'])
            self.car_controls['throttle'].run(direction['throttle'])

    def _get_preferred_directions(self, detected_object):
        """
        returns {(st, tr), ...}
        """
        result = set()

        segment_x, segment_y = self.detector_wrapper.get_detection_segment()
        close_enough = detected_object['vsize'] * detected_object['hsize'] > DETECTION_CLOSE_ENOUGH
        if segment_y in (-1, 1):
            if segment_x == -1:
                # -left, back
                for st in (0.5, 1.):
                    result.add((st, -1.))
            elif segment_x == 0:
                if close_enough:
                    return None  # stop
                else:
                    result.add((0., 1.))  # forward
            else:
                # -right, back
                for st in (-1., -0.5):
                    result.add((st, -1.))
        elif segment_y == 0:
            if segment_x == -1:
                if close_enough:
                    # -left, backward
                    for st in (0.5, 1.):
                        result.add((st, -1.))
                else:
                    # left, forward
                    for st in (-1., -0.5):
                        result.add((st, 1.))
            elif segment_x == 0:
                if close_enough:
                    return None  # stop
                else:
                    result.add((0., 1.))  # forward
            else:  # 1
                if close_enough:
                    # -right, backward
                    for st in (-1., -0.5):
                        result.add((st, -1.))
                else:
                    # right, forward
                    for st in (0.5, 1.):
                        result.add((st, 1.))

        return result

    def _store_track_frame(self, track_fh):
        weights = {f'({k[0]},{k[1]})': v for k, v in self.cached_weights.items()} if self.cached_weights else None
        frame = {
            'ts': self.move_ts,
            'pose_lag': self.pose_lag,
            'directions_weights': weights,
            'detected_object': self.detector_wrapper.cached_detected_object,
            'direction': self.cached_direction,
            'lidar': self.lidar.get_track_frame(),
            'tracker': self.tracker.get_track_frame(),
        }
        json.dump(frame, track_fh)
        track_fh.write('\n')
        track_fh.flush()
