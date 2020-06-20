from ballsbot.utils import keep_rps
from ballsbot.ndt import NDT
from math import pi, ceil, degrees
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.patches as patches
from io import BytesIO


class Tracker:
    def __init__(self, imu, lidar, odometry, fps=2, max_distance=15.):
        self.imu = imu
        self.lidar = lidar
        self.odometry = odometry
        self.fps = fps
        self.max_distance = max_distance
        self.poses = []
        self.ndt = NDT(grid_size=8., box_size=1., iterations_count=20, optim_step=(0.05, 0.05, 0.01), eps=0.01)
        self.readings = []

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

            if keep_readings:
                self.readings.append(current)

            if previous is None:
                self.poses.append({
                    'x': current['odometry_dx'],
                    'y': current['odometry_dy'],
                    'teta': current['teta'],
                })
            else:
                dt = current['ts'] - previous['ts']
                if dt == 0.:
                    # print('{} same ts {} - {} = {}'.format(ts, current['ts'], previous['ts'], dt))  # FIXME
                    continue
                current['odometry_dx'], current['odometry_dy'] = self.odometry.get_dx_dy(
                    dt, current['teta'], keep_readings=keep_readings
                )

                dx, dy, teta = self._get_transformation(dt, current, previous)
                prev_pose = self.poses[-1]
                self.poses.append({
                    'x': prev_pose['x'] + dx,
                    'y': prev_pose['y'] + dy,
                    'teta': teta,
                })
                # print('x = {:0.04f}, y = {:0.04f}, teta = {:0.04f}, '.format(self.poses[-1]['x'], self.poses[-1]['y'], self.poses[-1]['teta']))  # FIXME

            previous = current

    def _get_transformation(self, dt, current, previous):
        dteta_raw = current['teta'] - previous['teta']
        dx_raw = current['odometry_dx']
        dy_raw = current['odometry_dy']
        raw_result = (dx_raw, dy_raw, dteta_raw)
        dx, dy, dteta, converged, score = self.ndt.get_optimal_transformation(
            previous['points'],
            current['points'],
            start_point=[dx_raw, dy_raw, dteta_raw]
        )
        # print('dx = {:+0.04f}, dy = {:+0.04f}, dteta = {:+0.04f}, converged = {:0.0f}, score = {:0.04f}'.format(
        #     dx, dy, dteta, converged, score))  # FIXME

        if converged != 1.:
            # print('not converged')  # FIXME
            return raw_result

        if score > 0.5:
            # print('score > 0.1 ({:0.04f})'.format(score))  # FIXME
            return raw_result

        steps = ceil(dt / self.fps)

        max_dteta = steps * pi / 8
        if max_dteta > 2 * pi:
            max_dteta = 2 * pi
        if abs(dteta - dteta_raw) > max_dteta:
            # print('teta: {} - {} > {}'.format(dteta, dteta_raw, max_dteta))  # FIXME
            return raw_result

        prev_pose = self.poses[-1]
        max_dxy = dt * 0.2  # m/s
        if abs(dx_raw - dx) > max_dxy or abs(prev_pose['x'] + dx) > self.max_distance:
            # print('x: {} > {}'.format(dx, max_dxy))  # FIXME
            return raw_result
        if abs(dy_raw - dy) > max_dxy or abs(prev_pose['y'] + dy) > self.max_distance:
            # print('y: {} > {}'.format(dy, max_dxy))  # FIXME
            return raw_result

        teta = prev_pose['teta'] + dteta
        if teta >= 2 * pi:
            teta -= 2 * pi
        elif teta <= -2 * pi:
            teta += 2 * pi

        return dx, dy, teta

    def update_picture(self, image, only_nearby_meters=10):
        poses_x_points = [x['x'] for x in self.poses]
        poses_y_points = [x['y'] for x in self.poses]
        pose = self.poses[-1]
        points = self.ndt.apply_transformation_to_cloud(
            self.lidar.points,
            [pose['x'], pose['y'], pose['teta']]
        )
        xy, w, h = self.lidar._calibration_to_xywh(self.lidar.calibration)
        x, y = xy

        fig = Figure(figsize=(6, 5))
        canvas = FigureCanvas(fig)
        ax = fig.gca()

        ax.scatter(poses_x_points, poses_y_points, marker='o', s=10, c='gray')

        lidar_x_points = [x[0] for x in points]
        lidar_y_points = [x[1] for x in points]
        ax.scatter(lidar_x_points, lidar_y_points, marker='o', s=5, c='b')

        rect = patches.Rectangle(
            (x + pose['x'], y + pose['y']), w, h,
            linewidth=3, edgecolor='r', facecolor='none', angle=degrees(pose['teta'])
        )
        ax.add_patch(rect)

        ax.text(
            -only_nearby_meters, -only_nearby_meters,
            'x: {:0.02f}, y:{:0.02f}, teta: {:0.02f}'.format(pose['x'], pose['y'], pose['teta']),
            fontsize=10
        )

        ax.set_xlim(-only_nearby_meters, only_nearby_meters)
        ax.set_ylim(-only_nearby_meters, only_nearby_meters)
        ax.grid(which='both', linestyle='--', alpha=0.5)

        canvas.draw()
        jpeg = BytesIO()
        canvas.print_jpg(jpeg)
        image.value = jpeg.getvalue()
