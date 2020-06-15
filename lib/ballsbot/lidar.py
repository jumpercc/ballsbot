from math import cos, sin, pi
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.patches as patches
from io import BytesIO
import sys
from time import time

from ballsbot.utils import keep_rps

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

import rospy
from sensor_msgs.msg import LaserScan


def radial_to_cartesian(magnitude, angle):
    x = magnitude * cos(angle)
    y = magnitude * sin(angle)
    return [x, y]


class Lidar:
    def __init__(self):
        self.calibration = self._default_calibration()
        self.angle_min = -pi
        self.angle_max = pi
        self.points = []
        self.points_ts = time()

    def _default_calibration(self):
        return {
            'angle_fix': -1.660291627049446,
            'fl_x': -0.1086444572707752,
            'fl_y': 0.14257259585251014,
            'rr_x': 0.10419355369523223,
            'rr_y': -0.14585700720960665,
        }

    def _get_raw_lidar_points(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                self.angle_min = data.angle_min
                self.angle_max = data.angle_max
            except KeyboardInterrupt:
                return None
            except Exception as e:
                break
        return data

    def _fix_angle(self, my_angle, angle_fix=None):
        if angle_fix is None:
            angle_fix = self.calibration['angle_fix']
        my_angle -= angle_fix
        if my_angle < self.angle_min:
            my_angle = self.angle_max + my_angle - self.angle_min
        elif my_angle > self.angle_max:
            my_angle = self.angle_min + my_angle - self.angle_max
        return my_angle

    def _get_lidar_points(self, range_limit=None):
        data = self._get_raw_lidar_points()
        self.points_ts = time()
        if data is None:
            return None, None
        points = []
        angle = data.angle_min
        for i in range(len(data.intensities)):
            if data.intensities[i] > 0 and (range_limit is None or data.ranges[i] <= range_limit):
                a_point = radial_to_cartesian(data.ranges[i], self._fix_angle(angle))
                points.append(a_point)
            angle += data.angle_increment
        return points

    @staticmethod
    def _calibration_to_xywh(calibration):
        if calibration['fl_x'] > calibration['rr_x']:
            x = calibration['rr_x']
            w = calibration['fl_x'] - calibration['rr_x']
        else:
            x = calibration['fl_x']
            w = calibration['rr_x'] - calibration['fl_x']

        if calibration['fl_y'] > calibration['rr_y']:
            y = calibration['rr_y']
            h = calibration['fl_y'] - calibration['rr_y']
        else:
            y = calibration['fl_y']
            h = calibration['rr_y'] - calibration['fl_y']

        return (x, y), w, h

    def _update_picture(self, image, points, only_nearby_meters=4, additional_points=None):
        fig = Figure(figsize=(6, 5))
        canvas = FigureCanvas(fig)
        ax = fig.gca()

        x_points = [x[0] for x in points]
        y_points = [x[1] for x in points]
        ax.scatter(x_points, y_points, marker='o', s=5, c='b')

        if self.calibration is None:
            ax.scatter([0], [0], marker='o', s=50, c='r')
        else:
            rect = patches.Rectangle(
                *self._calibration_to_xywh(self.calibration), linewidth=3, edgecolor='r', facecolor='none'
            )
            ax.add_patch(rect)

        if additional_points is not None:
            ax.scatter(additional_points['x'], additional_points['y'], marker='o', s=25, c='g')

        ax.set_xlim(-only_nearby_meters, only_nearby_meters)
        ax.set_ylim(-only_nearby_meters, only_nearby_meters)
        ax.grid(which='both', linestyle='--', alpha=0.5)

        canvas.draw()
        jpeg = BytesIO()
        canvas.print_jpg(jpeg)
        image.value = jpeg.getvalue()

    def show_lidar_cloud(self, image, **kwargs):
        self.auto_update_lidar_cloud(
            cb=lambda points, only_nearby_meters: self._update_picture(
                image, points, only_nearby_meters=only_nearby_meters
            ),
            **kwargs
        )

    def auto_update_lidar_cloud(self, only_nearby_meters=4, fps=2, cb=None):
        ts = None
        while True:
            ts = keep_rps(ts, fps=fps)
            points = self._get_lidar_points()
            if points is None:
                self.points = []
                break
            else:
                self.points = points

            if cb is not None:
                cb(points, only_nearby_meters=only_nearby_meters)

    def _get_my_corners(self):
        range_limit = 0.3

        while True:
            data = self._get_raw_lidar_points()
            candidates = []
            distance_up = False
            wait_for_max = True
            angle = data.angle_min
            prev_range = None
            points_met = 0
            for i in range(len(data.intensities)):
                if data.intensities[i] > 0 and (range_limit is None or data.ranges[i] <= range_limit):
                    if prev_range is not None and abs(data.ranges[i] - prev_range) <= 0.03:  # denoise
                        x, y = radial_to_cartesian(data.ranges[i], angle)
                        if data.ranges[i] > prev_range:
                            distance_up = True
                            c = {
                                'range': data.ranges[i],
                                'angle': angle,
                                'x': x,
                                'y': y,
                                'index': i,
                                'points_met': points_met,
                            }
                            points_met = 0
                            if wait_for_max and len(candidates) and abs(candidates[-1]['angle'] - c['angle']) < 0.05:
                                c['points_met'] += candidates[-1]['points_met']
                                candidates[-1] = c
                            else:
                                candidates.append(c)
                            wait_for_max = True
                        else:
                            wait_for_max = False
                            if distance_up or len(candidates) == 0:
                                candidates.append({
                                    'range': data.ranges[i],
                                    'angle': angle,
                                    'x': x,
                                    'y': y,
                                    'index': i,
                                    'points_met': points_met,
                                })
                                points_met = 0
                            distance_up = False
                        points_met += 1
                    prev_range = data.ranges[i]
                angle += data.angle_increment

            corners = []
            max_one = None
            points_met = 0
            for c in candidates:
                points_met += c['points_met']
                if max_one is not None:
                    diff = abs(max_one['angle'] - c['angle'])
                    if diff > data.angle_max:
                        diff -= data.angle_max
                    if diff > 0.9:
                        corners.append(max_one)
                        points_met = points_met - max_one['points_met']
                        max_one = None
                c['points_met'] = points_met
                if max_one is None or max_one['range'] < c['range']:
                    max_one = c
            if max_one is not None:
                corners.append(max_one)

            if len(corners) == 4:
                break
            elif len(corners) == 3:
                pass  # TODO
        return corners, data

    def calibrate(self):
        corners, data = self._get_my_corners()

        points_met = 100000000
        front_left_index = 0
        for i, c in enumerate(corners):
            if c['points_met'] < points_met:
                front_left_index = i
                points_met = c['points_met']
        rear_right_index = (front_left_index + 2) % 4
        front_right_index = (front_left_index + 3) % 4

        fr_angle = corners[front_right_index]['angle']
        rr_angle = corners[rear_right_index]['angle']
        angle_fix = (fr_angle + rr_angle) / 2

        fl_x, fl_y = radial_to_cartesian(
            corners[front_left_index]['range'],
            self._fix_angle(corners[front_left_index]['angle'], angle_fix=angle_fix)
        )

        rr_x, rr_y = radial_to_cartesian(
            corners[rear_right_index]['range'],
            self._fix_angle(corners[rear_right_index]['angle'], angle_fix=angle_fix)
        )

        self.calibration = {
            'angle_fix': angle_fix,
            'fl_x': fl_x,
            'fl_y': fl_y,
            'rr_x': rr_x,
            'rr_y': rr_y,
        }
