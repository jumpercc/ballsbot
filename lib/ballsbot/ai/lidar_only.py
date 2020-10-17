from math import pi

from ballsbot.lidar import Lidar
from ballsbot.servos import get_controls
from ballsbot.utils import keep_rps
from ballsbot.cloud_to_lines import distance
from ballsbot.odometry import Odometry
from ballsbot.imu import IMU_Threaded


class LidarOnlyAI:
    TURN_RADIUS = 0.88
    CHECK_RADIUS = 0.5
    FROM_LIDAR_TO_CENTER = 0.07
    FEAR_DISTANCE = 0.05
    CAR_WIDTH = 0.18
    HALF_CAR_WIDTH = CAR_WIDTH / 2
    STOP = {'steering': 0., 'throttle': 0.}
    FORWARD_THROTTLE = 0.5
    BACKWARD_TROTTLE = -0.5
    FORWARD_BRAKE = -0.4
    BACKWARD_BRAKE = 0.4
    RIGHT = 1.
    LEFT = -1.

    def __init__(self, test_run=False):
        self.lidar = Lidar(test_run)
        self.BODY_POSITION = self.lidar.calibration_to_xywh(self.lidar.calibration)
        if not test_run:
            self.car_controls = get_controls()
            self.imu = IMU_Threaded()
            self.odometry = Odometry(self.imu, self.car_controls['throttle'])

    def run(self):
        ts = None
        direction = self.STOP
        steps_with_direction = 0
        while True:
            ts = keep_rps(ts, fps=4)
            prev_direction = direction
            direction = self._get_free_direction(direction, steps_with_direction)
            print(direction)
            self._follow_direction(direction)

            if prev_direction == direction:
                steps_with_direction += 1
            else:
                steps_with_direction = 0

    def _can_move_straight_forward(self, nearby_points):
        min_y = self.BODY_POSITION['y'] - self.FEAR_DISTANCE
        max_y = self.BODY_POSITION['y'] + self.BODY_POSITION['h'] + self.FEAR_DISTANCE
        min_x = self.BODY_POSITION['x'] + self.BODY_POSITION['w']
        max_x = min_x + self.CHECK_RADIUS

        def in_forward_direction(a_point):
            if min_x < a_point[0] < max_x and min_y <= a_point[1] <= max_y:
                return True
            return False

        return len(list(filter(in_forward_direction, nearby_points))) == 0

    def _can_move_straight_backward(self, nearby_points):
        min_y = self.BODY_POSITION['y'] - self.FEAR_DISTANCE
        max_y = self.BODY_POSITION['y'] + self.BODY_POSITION['h'] + self.FEAR_DISTANCE
        max_x = self.BODY_POSITION['x']
        min_x = max_x - self.CHECK_RADIUS

        def in_backward_direction(a_point):
            if min_x < a_point[0] < max_x and min_y <= a_point[1] <= max_y:
                return True
            return False

        return len(list(filter(in_backward_direction, nearby_points))) == 0

    def _filter_right_points(self, nearby_points):
        left_center, right_center, column_radius = self._get_columns()
        outer_radius = column_radius + 2 * (self.HALF_CAR_WIDTH + self.FEAR_DISTANCE)

        def on_a_curve(a_point):
            return distance(right_center, a_point) < outer_radius

        return list(filter(on_a_curve, nearby_points))

    def _filter_left_points(self, nearby_points):
        left_center, right_center, column_radius = self._get_columns()
        outer_radius = column_radius + 2 * (self.HALF_CAR_WIDTH + self.FEAR_DISTANCE)

        def on_a_curve(a_point):
            return distance(left_center, a_point) < outer_radius

        return list(filter(on_a_curve, nearby_points))

    def _can_move_right_forward(self, nearby_points):
        nearby_points = self._filter_right_points(nearby_points)
        min_x = self.BODY_POSITION['x'] + self.BODY_POSITION['w']

        def in_forward_direction(a_point):
            return min_x < a_point[0]

        return len(list(filter(in_forward_direction, nearby_points))) == 0

    def _can_move_right_backward(self, nearby_points):
        nearby_points = self._filter_right_points(nearby_points)
        max_x = self.BODY_POSITION['x']

        def in_backward_direction(a_point):
            return a_point[0] < max_x

        return len(list(filter(in_backward_direction, nearby_points))) == 0

    def _can_move_left_forward(self, nearby_points):
        nearby_points = self._filter_left_points(nearby_points)
        min_x = self.BODY_POSITION['x'] + self.BODY_POSITION['w']

        def in_forward_direction(a_point):
            return min_x < a_point[0]

        return len(list(filter(in_forward_direction, nearby_points))) == 0

    def _can_move_left_backward(self, nearby_points):
        nearby_points = self._filter_left_points(nearby_points)
        max_x = self.BODY_POSITION['x']

        def in_backward_direction(a_point):
            return a_point[0] < max_x

        return len(list(filter(in_backward_direction, nearby_points))) == 0

    def _get_free_direction(self, prev_direction, steps_with_direction):
        if prev_direction['throttle'] == self.FORWARD_BRAKE or prev_direction['throttle'] == self.BACKWARD_BRAKE:
            if self.odometry.direction > 0. and prev_direction['throttle'] == self.FORWARD_BRAKE \
                    or self.odometry.direction < 0. and prev_direction['throttle'] == self.BACKWARD_BRAKE:
                return prev_direction
            else:
                return self.STOP
        elif prev_direction['throttle'] == self.FORWARD_THROTTLE or prev_direction['throttle'] == self.BACKWARD_TROTTLE:
            if self.odometry.direction == 0. and steps_with_direction > 3:  # stop when jammed or on driver error
                return self.STOP

        nearby_points = self._get_nearby_points()

        can_move_straight_forward = self._can_move_straight_forward(nearby_points)
        can_move_right_forward = self._can_move_right_forward(nearby_points)
        can_move_left_forward = self._can_move_left_forward(nearby_points)

        can_move_straight_backward = self._can_move_straight_backward(nearby_points)
        can_move_right_backward = self._can_move_right_backward(nearby_points)
        can_move_left_backward = self._can_move_left_backward(nearby_points)

        if prev_direction['throttle'] == self.FORWARD_THROTTLE or prev_direction['throttle'] == 0.:
            if can_move_straight_forward:
                return {'steering': 0., 'throttle': self.FORWARD_THROTTLE}
            elif can_move_right_forward:
                return {'steering': self.RIGHT, 'throttle': self.FORWARD_THROTTLE}
            elif can_move_left_forward:
                return {'steering': self.LEFT, 'throttle': self.FORWARD_THROTTLE}
            elif prev_direction['throttle'] == self.FORWARD_THROTTLE:
                return {'steering': 0., 'throttle': self.FORWARD_BRAKE}
        if prev_direction['throttle'] == self.BACKWARD_TROTTLE or prev_direction['throttle'] == 0.:
            if can_move_straight_backward:
                return {'steering': 0., 'throttle': self.BACKWARD_TROTTLE}
            elif can_move_right_backward:
                return {'steering': self.RIGHT, 'throttle': self.BACKWARD_TROTTLE}
            elif can_move_left_backward:
                return {'steering': self.LEFT, 'throttle': self.BACKWARD_TROTTLE}
            elif prev_direction['throttle'] == self.BACKWARD_TROTTLE:
                return {'steering': 0., 'throttle': self.BACKWARD_BRAKE}

        return self.STOP

    def _get_columns(self):
        return [0., self.TURN_RADIUS], [0., -self.TURN_RADIUS], \
               self.TURN_RADIUS - self.HALF_CAR_WIDTH - self.FEAR_DISTANCE

    def _get_nearby_points(self):
        range_limit = self.CHECK_RADIUS + self.HALF_CAR_WIDTH + self.FEAR_DISTANCE
        range_limit += abs(self.FROM_LIDAR_TO_CENTER)
        nearby_points = self.lidar.get_radial_lidar_points(range_limit)

        nearby_points = list(filter(self._ellipse_like_range_filter, nearby_points))

        nearby_points = self.lidar.radial_points_to_cartesian(nearby_points)
        left_center, right_center, column_radius = self._get_columns()

        def not_in_columns_and_a_car(a_point):
            if a_point[1] > 0.:
                if distance(left_center, a_point) < column_radius:
                    return False
            elif a_point[1] < 0.:
                if distance(right_center, a_point) < column_radius:
                    return False

            if self.BODY_POSITION['x'] <= a_point[0] <= self.BODY_POSITION['x'] + self.BODY_POSITION['w'] \
                    and self.BODY_POSITION['y'] <= a_point[1] <= self.BODY_POSITION['y'] + self.BODY_POSITION['h']:
                return False

            return True

        return list(filter(not_in_columns_and_a_car, nearby_points))

    def _ellipse_like_range_filter(self, a_point):
        angle = a_point['angle']
        angle %= 2 * pi
        if angle < 0:
            angle = -angle

        if pi / 2 <= angle <= 3 * pi / 2:
            lidar_to_center = 0.
        else:
            lidar_to_center = self.FROM_LIDAR_TO_CENTER

        if angle >= pi:
            angle -= pi
        if angle > pi / 2:
            angle = pi - angle

        range_limit = self.CHECK_RADIUS + self.FEAR_DISTANCE \
                      + self.HALF_CAR_WIDTH * angle + lidar_to_center * (pi / 2 - angle)
        return a_point['distance'] < range_limit

    def _follow_direction(self, direction):
        self.car_controls['steering'].run(direction['steering'])
        self.car_controls['throttle'].run(direction['throttle'])
