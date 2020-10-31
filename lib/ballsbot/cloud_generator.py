from math import pi, tan, atan

from ballsbot.lidar import radial_to_cartesian
from ballsbot.geometry import get_linear_coefs, distance, cross_point, get_radial_line


def straight_distance(a_point, a_wall):
    a, b, c = a_wall
    if a == 0:
        return abs(a_point[1] + c / b)
    elif b == 0:
        return abs(a_point[0] + c / a)
    else:
        raise ValueError()


def shorter_distance(car_point, radial_line, first_wall, second_wall):
    first_point = cross_point(radial_line, first_wall)
    second_point = cross_point(radial_line, second_wall)
    distances = [
        distance(car_point, first_point),
        distance(car_point, second_point),
    ]
    return list(sorted(distances, key=lambda x: abs(x)))[0]


def get_long_room():
    car_point = [1.5, -1.]
    d_angle = 2. * pi / 490.

    lt = [0., 0.]
    rb = [4., -2.]

    walls = [
        get_linear_coefs(lt, [rb[0], lt[1]]),
        get_linear_coefs([rb[0], lt[1]], rb),
        get_linear_coefs(rb, [lt[0], rb[1]]),
        get_linear_coefs([lt[0], rb[1]], lt),
    ]

    points = []
    angle = 0.
    while angle < 2. * pi:
        radial_line = get_radial_line(car_point, angle)
        if angle == 0. or angle == 2. * pi:
            dist = straight_distance(car_point, walls[1])
        elif 0 < angle < pi / 2.:
            dist = shorter_distance(car_point, radial_line, walls[0], walls[1])
        elif angle == pi / 2.:
            dist = straight_distance(car_point, walls[0])
        elif pi / 2. < angle < pi:
            dist = shorter_distance(car_point, radial_line, walls[0], walls[3])
        elif angle == pi:
            dist = straight_distance(car_point, walls[3])
        elif pi < angle < 3. / 2. * pi:
            dist = shorter_distance(car_point, radial_line, walls[2], walls[3])
        elif angle == 3. / 2. * pi:
            dist = straight_distance(car_point, walls[2])
        elif 3. / 2. * pi < angle < 2. * pi:
            dist = shorter_distance(car_point, radial_line, walls[1], walls[2])
        else:
            raise ValueError()

        points.append(radial_to_cartesian(dist, angle))

        angle += d_angle

    return points, {'x': car_point[0], 'y': car_point[1], 'teta': 0.}
