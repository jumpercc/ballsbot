from ballsbot.utils import keep_rps
from ballsbot.ndt import NDT
from math import pi, ceil
import ballsbot.drawing as drawing
import sys
import json


class Tracker:
    def __init__(self, imu, lidar, odometry, fps=2, max_distance=15., fix_pose_with_lidar=True):
        self.imu = imu
        self.lidar = lidar
        self.odometry = odometry
        self.fps = fps
        self.max_distance = max_distance
        self.poses = []
        self.fix_pose_with_lidar = fix_pose_with_lidar
        self.ndt = NDT(grid_size=8., box_size=1., iterations_count=20, optim_step=(0.05, 0.05, 0.01), eps=0.01)
        self.errors = []

    def start(self, keep_readings=False):
        previous = None
        ts = None
        while True:
            ts = keep_rps(ts, fps=self.fps)

            current = {
                'points': self.lidar.points,
                'ts': self.lidar.points_ts,
                'teta': self.imu.get_teta(),
                'odometry_dx': 0.,
                'odometry_dy': 0.,
            }

            if previous is None:
                self.poses.append({
                    'x': current['odometry_dx'],
                    'y': current['odometry_dy'],
                    'teta': current['teta'],
                })
            else:
                dt = current['ts'] - previous['ts']
                if dt == 0.:
                    continue
                try:
                    current['odometry_dx'], current['odometry_dy'] = self.odometry.get_dx_dy(
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
                if keep_readings:
                    self.poses[-1]['points'] = current['points']

            previous = current

    def _get_transformation(self, dt, current, previous):
        dteta_raw = current['teta'] - previous['teta']
        dx_raw = current['odometry_dx']
        dy_raw = current['odometry_dy']
        raw_result = (dx_raw, dy_raw, current['teta'])
        if not self.fix_pose_with_lidar:
            return raw_result

        dx, dy, dteta, converged, score = self.ndt.get_optimal_transformation(
            previous['points'],
            current['points'],
            start_point=[dx_raw, dy_raw, dteta_raw]
        )

        if converged != 1.:
            return raw_result

        if score > 0.5:
            return raw_result

        steps = ceil(dt / self.fps)

        max_dteta = steps * pi / 8
        if max_dteta > 2 * pi:
            max_dteta = 2 * pi
        if abs(dteta - dteta_raw) > max_dteta:
            return raw_result

        prev_pose = self.poses[-1]
        max_dxy = dt * 0.2  # m/s
        if abs(dx_raw - dx) > max_dxy or abs(prev_pose['x'] + dx) > self.max_distance:
            return raw_result
        if abs(dy_raw - dy) > max_dxy or abs(prev_pose['y'] + dy) > self.max_distance:
            return raw_result

        teta = prev_pose['teta'] + dteta
        if teta >= 2 * pi:
            teta -= 2 * pi
        elif teta <= -2 * pi:
            teta += 2 * pi

        return dx, dy, teta

    def update_picture(self, image, only_nearby_meters=10):
        pose = self.poses[-1]
        points = self.ndt.apply_transformation_to_cloud(
            self.lidar.points,
            [pose['x'], pose['y'], pose['teta']]
        )
        self_position = self.lidar.calibration_to_xywh(self.lidar.calibration)
        drawing.update_image_abs_coords(image, self.poses, points, self_position, only_nearby_meters)

    def poses_to_a_file(self, file_path):
        with open(file_path, 'w') as a_file:
            a_file.write(json.dumps(self.poses))
