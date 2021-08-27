from math import sqrt, tan, atan, pi
from ballsbot_routing import ballsbot_routing


def get_linear_coefs(p1, p2):
    a = p1[1] - p2[1]
    b = p2[0] - p1[0]
    c = p1[0] * p2[1] - p2[0] * p1[1]
    return a, b, c


def point_to_line_distance(p0, a, b, c):
    return abs(a * p0[0] + b * p0[1] + c) / sqrt(a * a + b * b)


distance = ballsbot_routing.distance


def normal_to_line_in_point(line_coefs, a_point):
    a, b, _ = line_coefs
    x0, y0 = a_point
    if b == 0.:
        return 0., 1., -y0
    else:
        b2_d_a2 = -a / b
        c2_d_a2 = -x0 + a / b * y0
        return 1., b2_d_a2, c2_d_a2


def get_two_n_radians_lines(angle, line_coefs, a_point):
    a, b, _ = line_coefs
    x0, y0 = a_point
    if a == 0.:
        a1, b1 = (tan(angle), 1.)
        a2, b2 = (tan(-angle), 1.)
    elif b == 0:
        a1, b1 = (tan(pi / 2 - angle), 1.)
        a2, b2 = (tan(pi / 2 + angle), 1.)
    else:
        a1 = tan(atan(a / b) + angle)
        b1 = 1.
        a2 = tan(atan(a / b) - angle)
        b2 = 1.
    c1 = -a1 * x0 - b1 * y0
    c2 = -a2 * x0 - b2 * y0

    return (a1, b1, c1), (a2, b2, c2)


def on_other_side(line1_coefs, p1, p2):  # pylint: disable=R0914
    a1, b1, c1 = line1_coefs
    a2, b2, c2 = get_linear_coefs(p1, p2)
    if b1 == 0. and b2 == 0.:
        return False
    elif b1 == 0.:
        x0 = -c1 / a1
        y0 = -(a2 * x0 + c2) / b2
    elif b2 == 0.:
        x0 = -c2 / a2
        y0 = -(a1 * x0 + c1) / b1
    else:
        nom = c1 / b1 - c2 / b2
        denom = a2 / b2 - a1 / b1
        if denom == 0.:
            return False
        else:
            x0 = nom / denom
            y0 = -(a1 * x0 + c1) / b1

    x1, y1 = p1
    x2, y2 = p2

    if x1 > x2:
        if not x1 > x0 > x2:
            return False
    else:
        if not x1 < x0 < x2:
            return False

    if y1 > y2:
        if not y1 > y0 > y2:
            return False
    else:
        if not y1 < y0 < y2:
            return False

    return True


def get_radial_line(center_point, angle):
    a, b = (tan(angle), 1.)
    c = -a * center_point[0] - b * center_point[1]
    return a, b, c


def cross_point(first_line, second_line):
    a1, b1, c1 = first_line
    a2, b2, c2 = second_line

    if b1 == 0. and b2 == 0.:
        raise NotImplementedError()
    elif b1 == 0.:
        x = - c1 / a1
        y = -a2 / b2 * x - c2 / b2
    elif b2 == 0.:
        x = - c2 / a2
        y = -a1 / b1 * x - c1 / b1
    else:
        x = (c1 / b1 - c2 / b2) / (a2 / b2 - a1 / b1)
        y = -a1 / b1 * x - c1 / b1
    return x, y
