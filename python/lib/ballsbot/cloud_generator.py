from math import pi

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


def shorter_distance(car_point, radial_line, *walls):
    points = [cross_point(radial_line, x) for x in walls]
    distances = [distance(car_point, x) for x in points]
    return list(sorted(distances, key=lambda x: abs(x)))[0]  # pylint: disable=W0108


def shorter_distance_corner(car_point, radial_line, corner_wall, corner, *walls):
    check_point = cross_point(radial_line, corner_wall)
    dist = distance(car_point, check_point)
    if dist <= distance(car_point, corner):
        return dist
    else:
        return shorter_distance(car_point, radial_line, *walls)


def get_long_room():
    car_point = [1.5, -1.]
    car_teta = 0.

    lt = [0., 0.]
    rb = [4., -2.]

    walls = [
        get_linear_coefs(lt, [rb[0], lt[1]]),
        get_linear_coefs([rb[0], lt[1]], rb),
        get_linear_coefs(rb, [lt[0], rb[1]]),
        get_linear_coefs([lt[0], rb[1]], lt),
    ]

    d_angle = 2. * pi / 490.
    points = []
    radial_points = []
    angle = 0.
    while angle < 2. * pi:
        radial_line = get_radial_line(car_point, angle)
        if angle in (0., 2. * pi):
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

        fixed_angle = angle - car_teta
        if fixed_angle < -pi:
            fixed_angle += 2. * pi
        radial_points.append({'distance': dist, 'angle': fixed_angle})
        points.append(radial_to_cartesian(dist, fixed_angle))

        angle += d_angle

    return points, radial_points, {'x': car_point[0], 'y': car_point[1], 'teta': car_teta}


def get_g_room():
    car_point = [1.25, -1.75]
    car_teta = -pi / 6.

    lt = [0., 0.]
    rb = [4., -4.]
    cr = [2., -2.]

    walls = [
        get_linear_coefs(lt, [rb[0], lt[1]]),
        get_linear_coefs([rb[0], lt[1]], rb),
        get_linear_coefs(rb, [cr[0], rb[1]]),
        get_linear_coefs([cr[0], rb[1]], cr),
        get_linear_coefs(cr, [lt[0], cr[1]]),
        get_linear_coefs([lt[0], cr[1]], lt),
    ]

    d_angle = 2. * pi / 490.
    points = []
    radial_points = []
    angle = 0.
    while angle < 2. * pi:
        radial_line = get_radial_line(car_point, angle)
        if angle in (0., 2. * pi):
            dist = straight_distance(car_point, walls[1])
        elif 0 < angle < pi / 2.:
            dist = shorter_distance(car_point, radial_line, walls[0], walls[1])
        elif angle == pi / 2.:
            dist = straight_distance(car_point, walls[0])
        elif pi / 2. < angle < pi:
            dist = shorter_distance(car_point, radial_line, walls[0], walls[5])
        elif angle == pi:
            dist = straight_distance(car_point, walls[5])
        elif pi < angle < 3. / 2. * pi:
            dist = shorter_distance(car_point, radial_line, walls[4], walls[5])
        elif angle == 3. / 2. * pi:
            dist = straight_distance(car_point, walls[4])
        elif 3. / 2. * pi < angle < 2. * pi:
            dist = shorter_distance_corner(car_point, radial_line, walls[4], cr, walls[1], walls[2])
        else:
            raise ValueError()

        fixed_angle = angle - car_teta
        if fixed_angle < -pi:
            fixed_angle += 2. * pi
        radial_points.append({'distance': dist, 'angle': fixed_angle})
        points.append(radial_to_cartesian(dist, fixed_angle))

        angle += d_angle

    return points, radial_points, {'x': car_point[0], 'y': car_point[1], 'teta': car_teta}
