from math import sin, cos, pi
import sys
import json

from ballsbot.utils import keep_rps
# from ballsbot.ndt import NDT
from ballsbot import drawing
from ballsbot.lidar import apply_transformation_to_cloud
from ballsbot.odometry import Odometry
from ballsbot.imu import IMU


class TrackerLight:
    def __init__(self, fps=20):
        self.imu = IMU()
        self.odometry = Odometry()
        self.fps = fps
        self.poses = []
        self.errors = []
        self.teta_eps = 0.01
        self.prev = None

    def start(self):
        previous = None
        ts = None
        while True:
            ts = keep_rps(ts, fps=self.fps)

            current = self._get_current()

            if previous is None:
                self.poses.append({
                    'x': current['odometry_dx'],
                    'y': current['odometry_dy'],
                    'teta': current['teta'],
                })
            else:
                dt = current['ts'] - previous['ts']  # pylint: disable=E1136
                if dt == 0.:
                    continue
                try:
                    current['odometry_dx'], current['odometry_dy'] = self._get_dx_dy(
                        dt, current['teta']
                    )
                except Exception as e:
                    self.errors.append('odometry: {}'.format(e))
                    raise

                try:
                    dx, dy, teta = self._get_transformation(dt, current, previous)
                except Exception as e:
                    print('_get_transformation failed with {}'.format(e), file=sys.stderr)
                    raise

                prev_pose = self.poses[-1]
                self.poses.append({
                    'ts': current['ts'],
                    'x': prev_pose['x'] + dx,
                    'y': prev_pose['y'] + dy,
                    'teta': teta,
                })
                self._upgrade_current(current)

            previous = current

    def _get_dx_dy(self, dt, teta):
        current = {
            'teta': teta,
            'w_z': self.imu.get_w_z(),
            'dt': dt,
        }

        if self.prev is None:
            result = [0., 0.]
        else:
            dteta = teta - self.prev['teta']
            if dteta > pi:
                dteta -= 2 * pi
            elif dteta < -pi:
                dteta += 2 * pi
            current['dteta'] = dteta

            if self.odometry.get_speed() == 0. or self.odometry.direction == 0:
                result = [0., 0.]
            else:
                speed = self.odometry.direction * self.odometry.get_speed()
                if abs(dteta) < self.teta_eps:
                    dx = speed * cos(teta) * dt
                    dy = speed * sin(teta) * dt
                else:
                    w_z = current['w_z']
                    prev_teta = self.prev['teta']
                    if w_z == 0.:
                        dx = dy = 0.
                    else:
                        dx = speed / w_z * (-sin(prev_teta) + sin(prev_teta + w_z * dt))
                        dy = speed / w_z * (cos(prev_teta) - cos(prev_teta + w_z * dt))
                result = [dx, dy]

        self.prev = current
        return result

    def _get_current(self):
        return {
            'ts': self.imu.get_teta_ts(),
            'teta': self.imu.get_teta(),
            'odometry_dx': 0.,
            'odometry_dy': 0.,
        }

    def _upgrade_current(self, current):
        pass

    def _get_transformation(self, dt, current, previous):  # pylint: disable=R0201, W0613
        dx_raw = current['odometry_dx']
        dy_raw = current['odometry_dy']
        raw_result = (dx_raw, dy_raw, current['teta'])
        return raw_result

    def update_picture(self, image, only_nearby_meters=10):
        drawing.update_image_abs_coords(image, self.poses, [], None, only_nearby_meters)

    def poses_to_a_file(self, file_path):
        with open(file_path, 'w') as a_file:
            a_file.write(json.dumps(self.poses))

    def get_current_pose(self):
        return self.poses[-1]


class Tracker(TrackerLight):
    # pylint: disable=R0913
    def __init__(self, lidar, fps=2, max_distance=15., fix_pose_with_lidar=False, keep_readings=False):
        super().__init__(fps)
        self.lidar = lidar
        self.max_distance = max_distance
        if fix_pose_with_lidar:
            raise NotImplementedError()
        self.fix_pose_with_lidar = fix_pose_with_lidar
        # self.ndt = NDT(grid_size=8., box_size=1., iterations_count=20, optim_step=(0.05, 0.05, 0.01), eps=0.01)
        self.keep_readings = keep_readings

    def _get_current(self):
        result = super()._get_current()
        result['points'] = self.lidar.get_lidar_points()
        result['ts'] = self.lidar.points_ts
        return result

    def _upgrade_current(self, current):
        self.poses[-1]['points'] = current['points']

    def _get_transformation(self, dt, current, previous):
        raw_result = super()._get_transformation(dt, current, previous)
        if not self.fix_pose_with_lidar:
            return raw_result
        return None  # FIXME
        # dteta_raw = current['teta'] - previous['teta']
        # dx_raw = current['odometry_dx']
        # dy_raw = current['odometry_dy']
        #
        # dx, dy, dteta, converged, score = self.ndt.get_optimal_transformation(
        #     previous['points'],
        #     current['points'],
        #     start_point=[dx_raw, dy_raw, dteta_raw]
        # )
        #
        # if converged != 1.:
        #     return raw_result
        #
        # if score > 0.5:
        #     return raw_result
        #
        # steps = ceil(dt / self.fps)
        #
        # max_dteta = steps * pi / 8
        # if max_dteta > 2 * pi:
        #     max_dteta = 2 * pi
        # if abs(dteta - dteta_raw) > max_dteta:
        #     return raw_result
        #
        # prev_pose = self.poses[-1]
        # max_dxy = dt * 0.2  # m/s
        # if abs(dx_raw - dx) > max_dxy or abs(prev_pose['x'] + dx) > self.max_distance:
        #     return raw_result
        # if abs(dy_raw - dy) > max_dxy or abs(prev_pose['y'] + dy) > self.max_distance:
        #     return raw_result
        #
        # teta = prev_pose['teta'] + dteta
        # if teta >= 2 * pi:
        #     teta -= 2 * pi
        # elif teta <= -2 * pi:
        #     teta += 2 * pi
        #
        # return dx, dy, teta

    def update_picture(self, image, only_nearby_meters=10):
        pose = self.poses[-1]
        points = apply_transformation_to_cloud(
            self.lidar.get_lidar_points(),
            [pose['x'], pose['y'], pose['teta']]
        )
        self_position = self.lidar.calibration_to_xywh(self.lidar.calibration)
        drawing.update_image_abs_coords(image, self.poses, points, self_position, only_nearby_meters)
