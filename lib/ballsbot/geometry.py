from math import sqrt


def get_linear_coefs(p1, p2):
    a = p1[1] - p2[1]
    b = p2[0] - p1[0]
    c = p1[0] * p2[1] - p2[0] * p1[1]
    return a, b, c


def point_to_line_distance(p0, a, b, c):
    return abs(a * p0[0] + b * p0[1] + c) / sqrt(a * a + b * b)


def distance(one, two):
    return sqrt((one[0] - two[0]) * (one[0] - two[0]) + (one[1] - two[1]) * (one[1] - two[1]))


def normal_to_line_in_point(line_coefs, a_point):
    a, b, c = line_coefs
    x0, y0 = a_point
    if b == 0.:
        return 0., 1., -y0
    else:
        b2_d_a2 = -a / b
        c2_d_a2 = -x0 + a / b * y0
        return 1., b2_d_a2, c2_d_a2


def get_45_degrees_lines(line_coefs, a_point):
    a, b, c = line_coefs
    x0, y0 = a_point
    if a == 0.:
        c /= b
        b = 1.
    else:
        b /= a
        c /= a
        a = 1.

    a1, b1 = (a + 1, b + 1)
    c1 = -a1 * x0 - b1 * y0
    a2, b2 = (a - 1, b - 1)
    c2 = -a2 * x0 - b2 * y0

    return (a1, b1, c1), (a2, b2, c2)
