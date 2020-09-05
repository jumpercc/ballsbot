from math import cos, sin, pi, atan2, sqrt
from ballsbot.cloud_to_lines import distance, point_to_line_distance


def coefs(a, b, c):
    if b == 0:
        coef_i = a
        coef_s = c
    else:
        coef_i = a / b
        coef_s = c / b
    return coef_i, coef_s


def get_lines_pairs(lines_one, lines_two):
    pairs = []
    for i, one in enumerate(lines_one):
        start_one, end_one, a_one, b_one, c_one = one
        coef_i_one, coef_s_one = coefs(a_one, b_one, c_one)
        rev_coef_i_one, rev_coef_s_one = coefs(b_one, a_one, c_one)

        for j, two in enumerate(lines_two):
            start_two, end_two, a_two, b_two, c_two = two
            coef_i_two, coef_s_two = coefs(a_two, b_two, c_two)
            rev_coef_i_two, rev_coef_s_two = coefs(b_two, a_two, c_two)

            diff_i = abs(coef_i_one - coef_i_two)
            diff_s = abs(coef_s_one - coef_s_two)
            rev_diff_i = abs(rev_coef_i_one - rev_coef_i_two)
            rev_diff_s = abs(rev_coef_s_one - rev_coef_s_two)
            if (diff_i > 1. or diff_s > 1.) and (rev_diff_i > 1. or rev_diff_s > 1.):
                continue

            dist_start = distance(start_one, start_two)
            dist_end = distance(end_one, end_two)
            if dist_start <= 1.1 or dist_end <= 1.1:
                pass  # same
            elif dist_start < distance(start_one, end_one) \
                    and dist_end < distance(start_two, end_two) \
                    and distance(end_one, start_two) > 0.3:
                pass  # intersected
            else:
                continue

            pairs.append([i, j, diff_i, diff_s, dist_start, dist_end])

    result = []
    if len(pairs):
        seen_left = set()
        seen_right = set()
        for it in sorted(pairs, key=lambda x: sqrt(x[2] * x[2] + x[3] * x[3])):
            if it[0] in seen_left:
                continue
            seen_left.add(it[0])
            if it[1] in seen_right:
                continue
            seen_right.add(it[1])
            result.append(it)

    return result


def get_line_position(a_line, self_position):
    start, end, a, b, c = a_line
    angle = atan2(a, b)
    dist = point_to_line_distance(self_position, a, b, c)
    return angle, dist


def get_coords_diff(lines_one, lines_two, position):
    pairs = get_lines_pairs(lines_one, lines_two)
    if len(pairs) == 0:
        return None
    list_d_angle = []
    list_dx = []
    list_dy = []
    list_legnths = []
    for it in pairs:
        angle_one, dist_one = get_line_position(lines_one[it[0]], position)
        angle_two, dist_two = get_line_position(lines_two[it[1]], position)

        d_angle = angle_two - angle_one
        if abs(d_angle) > pi / 4:
            continue
        if d_angle > pi:
            d_angle -= 2 * pi
        elif d_angle < -pi:
            d_angle += 2 * pi
        d_dist = dist_two - dist_one
        dx = d_dist * cos(pi / 2 - angle_one)
        dy = d_dist * sin(pi / 2 - angle_one)

        list_d_angle.append(d_angle)
        list_dx.append(dx)
        list_dy.append(dy)
        list_legnths.append(sqrt(
            (distance(lines_one[it[0]][0], lines_one[it[0]][1])
             + distance(lines_two[it[1]][0], lines_two[it[1]][1]))
            / 2.
        ))
        # print('{}-{}: dx={:+0.04f}, dy={:+0.04f}, dteta={:+0.04f} for {:+0.04f}'.format(
        #     it[0], it[1], dx, dy, d_angle, list_legnths[-1]))

    if len(list_d_angle) < 2:
        return None

    sum_len = sum(list_legnths)
    list_weights = [x / sum_len for x in list_legnths]

    avg_d_angle = 0.
    avg_dx = 0.
    avg_dy = 0.
    for i, weight in enumerate(list_weights):
        avg_d_angle += list_d_angle[i] * weight
        avg_dx += list_dx[i] * weight
        avg_dy += list_dy[i] * weight

    if len(list_d_angle) < 3 and (abs(avg_dx) > 1. or abs(avg_dy) > 1. or abs(avg_d_angle) > pi / 8):
        return None
    elif abs(avg_dx) > 2. or abs(avg_dy) > 2. or abs(avg_d_angle) > pi / 4:
        return None

    return avg_dx, avg_dy, avg_d_angle
