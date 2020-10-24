from math import pi

from ballsbot.lidar import Lidar
from ballsbot.servos import get_controls
from ballsbot.utils import keep_rps, run_as_thread
from ballsbot.cloud_to_lines import distance
from ballsbot.odometry import Odometry
from ballsbot.imu import IMU_Threaded
from ballsbot.tracking import Tracker


class Explorer:
    TURN_RADIUS = 0.88
    CHECK_RADIUS = 2.
    A_BIT_CENTER_Y = CHECK_RADIUS / 2. * 2.5
    STOP_DISTANCE = 0.3
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
    A_BIT_RIGHT = 0.5
    A_BIT_LEFT = -0.5
    INNER_OFFSET = 0.03

    def __init__(self, test_run=False):
        self.lidar = Lidar(test_run)
        self.BODY_POSITION = self.lidar.calibration_to_xywh(self.lidar.calibration)
        if not test_run:
            self.car_controls = get_controls()
            self.imu = IMU_Threaded()
            self.odometry = Odometry(self.imu, self.car_controls['throttle'])
            self.tracker = Tracker(self.imu, self.lidar, self.odometry, fps=4, fix_pose_with_lidar=False)

    def run(self):
        def tracker_run():
            self.tracker.start()

        run_as_thread(tracker_run)

        ts = None
        direction = self.STOP
        steps_with_direction = 0
        keep_for = 0
        while True:
            ts = keep_rps(ts, fps=4)
            prev_direction = direction
            if keep_for <= 0:
                direction, keep_for = self._get_free_direction(direction, steps_with_direction)
            keep_for -= 1
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
        stop_x = min_x + self.STOP_DISTANCE
        min_x -= self.INNER_OFFSET

        nearest_x = max_x
        for a_point in nearby_points:
            if min_x < a_point[0] < max_x and min_y <= a_point[1] <= max_y:
                if a_point[0] < stop_x:
                    return False, 0.
                elif nearest_x > a_point[0]:
                    nearest_x = a_point[0]
        return True, nearest_x

    def _can_move_straight_backward(self, nearby_points):
        min_y = self.BODY_POSITION['y'] - self.FEAR_DISTANCE
        max_y = self.BODY_POSITION['y'] + self.BODY_POSITION['h'] + self.FEAR_DISTANCE
        max_x = self.BODY_POSITION['x']
        min_x = max_x - self.CHECK_RADIUS
        stop_x = max_x - self.STOP_DISTANCE
        max_x += self.INNER_OFFSET

        nearest_x = min_x
        for a_point in nearby_points:
            if min_x < a_point[0] < max_x and min_y <= a_point[1] <= max_y:
                if a_point[0] > stop_x:
                    return False, 0.
                elif nearest_x < a_point[0]:
                    nearest_x = a_point[0]
        return True, -nearest_x

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

    def _filter_a_bit_left_points(self, nearby_points):
        left_center = [0, self.A_BIT_CENTER_Y]
        center_radius = self.A_BIT_CENTER_Y
        outer_radius = center_radius + self.HALF_CAR_WIDTH + self.FEAR_DISTANCE
        inner_radius = center_radius - self.HALF_CAR_WIDTH - self.FEAR_DISTANCE

        def on_a_curve(a_point):
            return inner_radius < distance(left_center, a_point) < outer_radius

        return list(filter(on_a_curve, nearby_points))

    def _filter_a_bit_right_points(self, nearby_points):
        right_center = [0, -self.A_BIT_CENTER_Y]
        center_radius = self.A_BIT_CENTER_Y
        outer_radius = center_radius + self.HALF_CAR_WIDTH + self.FEAR_DISTANCE
        inner_radius = center_radius - self.HALF_CAR_WIDTH - self.FEAR_DISTANCE

        def on_a_curve(a_point):
            return inner_radius < distance(right_center, a_point) < outer_radius

        return list(filter(on_a_curve, nearby_points))

    def _can_move_some_forward(self, nearby_points, a_filter):
        nearby_points = a_filter(nearby_points)
        min_x = self.BODY_POSITION['x'] + self.BODY_POSITION['w']
        max_x = min_x + self.CHECK_RADIUS
        stop_x = min_x + self.STOP_DISTANCE
        min_x -= self.INNER_OFFSET

        nearest_x = max_x
        for a_point in nearby_points:
            if min_x < a_point[0]:
                if a_point[0] < stop_x:
                    return False, 0.
                elif nearest_x > a_point[0]:
                    nearest_x = a_point[0]
        return True, nearest_x

    def _can_move_right_forward(self, nearby_points):
        return self._can_move_some_forward(nearby_points, self._filter_right_points)

    def _can_move_left_forward(self, nearby_points):
        return self._can_move_some_forward(nearby_points, self._filter_left_points)

    def _can_move_a_bit_right_forward(self, nearby_points):
        return self._can_move_some_forward(nearby_points, self._filter_a_bit_right_points)

    def _can_move_a_bit_left_forward(self, nearby_points):
        return self._can_move_some_forward(nearby_points, self._filter_a_bit_left_points)

    def _can_move_some_backward(self, nearby_points, a_filter):
        nearby_points = a_filter(nearby_points)
        max_x = self.BODY_POSITION['x']
        min_x = max_x - self.CHECK_RADIUS
        stop_x = max_x - self.STOP_DISTANCE
        max_x += self.INNER_OFFSET

        nearest_x = min_x
        for a_point in nearby_points:
            if a_point[0] < max_x:
                if a_point[0] > stop_x:
                    return False, 0.
                elif nearest_x < a_point[0]:
                    nearest_x = a_point[0]
        return True, -nearest_x

    def _can_move_right_backward(self, nearby_points):
        return self._can_move_some_backward(nearby_points, self._filter_right_points)

    def _can_move_left_backward(self, nearby_points):
        return self._can_move_some_backward(nearby_points, self._filter_left_points)

    def _can_move_a_bit_right_backward(self, nearby_points):
        return self._can_move_some_backward(nearby_points, self._filter_a_bit_right_points)

    def _can_move_a_bit_left_backward(self, nearby_points):
        return self._can_move_some_backward(nearby_points, self._filter_a_bit_left_points)

    def for_sort(self, direction):
        if direction == 0.:
            return 5
        elif direction == self.A_BIT_LEFT:
            return 4
        elif direction == self.A_BIT_RIGHT:
            return 3
        elif direction == self.LEFT:
            return 2
        else:
            return 1

    def _get_free_direction(self, prev_direction, steps_with_direction):
        if prev_direction['throttle'] == self.FORWARD_BRAKE or prev_direction['throttle'] == self.BACKWARD_BRAKE:
            if self.odometry.direction > 0. and prev_direction['throttle'] == self.FORWARD_BRAKE \
                    or self.odometry.direction < 0. and prev_direction['throttle'] == self.BACKWARD_BRAKE:
                return prev_direction, 1
            else:
                return self.STOP, 1
        elif prev_direction['throttle'] == self.FORWARD_THROTTLE or prev_direction['throttle'] == self.BACKWARD_TROTTLE:
            if self.odometry.direction == 0. and steps_with_direction > 3:  # stop when jammed or on driver error
                return self.STOP, 4

        nearby_points = self._get_nearby_points()

        forward = [
            [0.] + list(self._can_move_straight_forward(nearby_points)),
            [self.RIGHT] + list(self._can_move_right_forward(nearby_points)),
            [self.LEFT] + list(self._can_move_left_forward(nearby_points)),
            [self.A_BIT_RIGHT] + list(self._can_move_a_bit_right_forward(nearby_points)),
            [self.A_BIT_LEFT] + list(self._can_move_a_bit_left_forward(nearby_points)),
        ]
        forward = list(filter(lambda x: x[1], [[ft[0], ft[1], ft[2], self.for_sort(ft[0])] for ft in forward]))

        backward = [
            [0.] + list(self._can_move_straight_backward(nearby_points)),
            [self.RIGHT] + list(self._can_move_right_backward(nearby_points)),
            [self.LEFT] + list(self._can_move_left_backward(nearby_points)),
            [self.A_BIT_RIGHT] + list(self._can_move_a_bit_right_backward(nearby_points)),
            [self.A_BIT_LEFT] + list(self._can_move_a_bit_left_backward(nearby_points)),
        ]
        backward = list(filter(lambda x: x[1], [[ft[0], ft[1], ft[2], self.for_sort(ft[0])] for ft in backward]))

        if prev_direction['throttle'] == self.FORWARD_THROTTLE or prev_direction['throttle'] == 0.:
            if len(forward):
                # TODO sort by distance, not only x distance
                steering = sorted(forward, key=lambda x: (x[2], x[3]), reverse=True)[0][0]
                return {'steering': steering, 'throttle': self.FORWARD_THROTTLE}, 1
            elif prev_direction['throttle'] == self.FORWARD_THROTTLE:
                return {'steering': 0., 'throttle': self.FORWARD_BRAKE}, 1
        if prev_direction['throttle'] == self.BACKWARD_TROTTLE or prev_direction['throttle'] == 0.:
            if len(backward):
                steering = sorted(backward, key=lambda x: (x[2], x[3]), reverse=True)[0][0]
                return {'steering': steering, 'throttle': self.BACKWARD_TROTTLE}, 1
            elif prev_direction['throttle'] == self.BACKWARD_TROTTLE:
                return {'steering': 0., 'throttle': self.BACKWARD_BRAKE}, 1

        return self.STOP, 1

    def _get_columns(self):
        return [0., self.TURN_RADIUS], [0., -self.TURN_RADIUS], \
               self.TURN_RADIUS - self.HALF_CAR_WIDTH - self.FEAR_DISTANCE

    def _get_nearby_points(self):
        range_limit = self.CHECK_RADIUS + self.HALF_CAR_WIDTH + self.FEAR_DISTANCE
        range_limit += abs(self.FROM_LIDAR_TO_CENTER)
        nearby_points = self.lidar.get_radial_lidar_points(range_limit, cached=False)

        nearby_points = list(filter(self._ellipse_like_range_filter, nearby_points))

        nearby_points = self.lidar.radial_points_to_cartesian(nearby_points)
        left_center, right_center, column_radius = self._get_columns()

        def not_in_columns_and_a_car(a_point):
            if a_point[1] > 0.:
                if distance(left_center, a_point) < column_radius:
                    return False
                elif a_point[1] > left_center[1] \
                        and left_center[0] - column_radius < a_point[0] < left_center[0] + column_radius:
                    return False
            elif a_point[1] < 0.:
                if distance(right_center, a_point) < column_radius:
                    return False
                elif a_point[1] < right_center[1] \
                        and right_center[0] - column_radius < a_point[0] < right_center[0] + column_radius:
                    return False

            if self.BODY_POSITION['x'] + self.INNER_OFFSET <= a_point[0] <= \
                    self.BODY_POSITION['x'] + self.BODY_POSITION['w'] - self.INNER_OFFSET \
                    and self.BODY_POSITION['y'] + self.INNER_OFFSET <= a_point[1] <= \
                    self.BODY_POSITION['y'] + self.BODY_POSITION['h'] - self.INNER_OFFSET:
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
