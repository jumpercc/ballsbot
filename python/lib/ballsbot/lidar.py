from math import cos, sin, pi
import numpy as np
import ballsbot.drawing as drawing
import sys
from time import time
from random import random

from ballsbot.utils import keep_rps

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

import rospy
from sensor_msgs.msg import LaserScan


def radial_to_cartesian(magnitude, angle):
    x = magnitude * cos(angle)
    y = magnitude * sin(angle)
    return [x, y]


def apply_transformation_to_cloud(a_cloud, tr):
    result = []

    tx, ty, fi = tr
    rotate_m = np.array([
        [cos(fi), -sin(fi)],
        [sin(fi), cos(fi)]
    ])
    move_m = np.array([tx, ty]).reshape((2, 1))

    for point in a_cloud:
        point = np.array(point).reshape((2, 1))
        point = rotate_m @ point + move_m
        result.append(point[:, 0].tolist())

    return result


class TestLidarData:
    def __init__(self):
        self.angle_min = 0.
        self.angle_max = 2 * pi
        points_count = 5000
        self.intensities = [1. for _ in range(points_count)]
        self.ranges = [random() * 3. for _ in range(points_count)]
        self.angle_increment = (self.angle_max - self.angle_min) / points_count


class Lidar:
    def __init__(self, test_run=False):
        self.calibration = self._default_calibration()
        self.angle_min = -pi
        self.angle_max = pi
        self.radial_points = []
        self.points = []
        self.points_ts = time()
        self.test_run = test_run

    def _default_calibration(self):
        return {
            'angle_fix': 3.0810609316737434,
            'fl_x': 0.2041686441507201,
            'fl_y': 0.10426277741236079,
            'rr_x': -0.08011094659163859,
            'rr_y': -0.0988675829003773,
        }

    def _get_raw_lidar_points(self):
        data = None
        if not self.test_run:
            while data is None:
                try:
                    data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                    self.angle_min = data.angle_min
                    self.angle_max = data.angle_max
                except KeyboardInterrupt:
                    return None
                except Exception as e:
                    break
        else:
            data = TestLidarData()
            self.angle_min = data.angle_min
            self.angle_max = data.angle_max

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

    def get_radial_lidar_points(self, range_limit=None, cached=True):
        if not cached:
            self._get_radial_lidar_points()
        if range_limit is None:
            return self.radial_points
        return list(filter(lambda x: x['distance'] <= range_limit, self.radial_points))

    def _get_radial_lidar_points(self):
        data = self._get_raw_lidar_points()
        self.points_ts = time()
        if data is None:
            self.radial_points = []
            return self.radial_points
        points = []
        angle = data.angle_min
        for i in range(len(data.intensities)):
            if data.intensities[i] > 0:
                points.append({'distance': data.ranges[i], 'angle': self._fix_angle(angle)})
            angle += data.angle_increment

        self.radial_points = points
        self.points = []
        return points

    @staticmethod
    def radial_points_to_cartesian(points):
        return [radial_to_cartesian(x['distance'], x['angle']) for x in points]

    def get_lidar_points(self):
        if len(self.points) == 0 and len(self.radial_points) > 0:
            self.points = self.radial_points_to_cartesian(self.radial_points)  # FIXME race condition?
        return self.points

    def _get_lidar_points(self):
        points = self._get_radial_lidar_points()
        points = self.radial_points_to_cartesian(points)
        self.points = points
        return points

    @staticmethod
    def calibration_to_xywh(calibration):
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

        return {'x': x, 'y': y, 'w': w, 'h': h, }

    def _update_picture(self, image, points, only_nearby_meters=4, additional_points=None):
        if self.calibration is None:
            self_position = None
        else:
            self_position = self.calibration_to_xywh(self.calibration)
        drawing.update_picture_self_coords(image, points, self_position, only_nearby_meters, additional_points)

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
            if len(points) == 0:
                break

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
        fl_angle = corners[front_left_index]['angle']
        angle_fix = (fl_angle + fr_angle) / 2

        if corners[front_left_index]['x'] < corners[rear_right_index]['x']:
            angle_fix += pi

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
