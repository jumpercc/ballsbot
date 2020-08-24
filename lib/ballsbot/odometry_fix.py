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
            if (diff_i > 15. or diff_s > 30.) and (rev_diff_i > 15. or rev_diff_s > 30.):
                continue

            dist_start = distance(start_one, start_two)
            dist_end = distance(end_one, end_two)
            if dist_start > 0.6 or dist_end > 0.6:
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
    avg_d_angle = 0.
    avg_dx = 0.
    avg_dy = 0.
    ok_pairs_count = 0
    for it in pairs:
        angle_one, dist_one = get_line_position(lines_one[it[0]], position)
        angle_two, dist_two = get_line_position(lines_two[it[1]], position)

        d_angle = angle_two - angle_one
        if d_angle > pi:
            d_angle -= 2 * pi
        elif d_angle < -pi:
            d_angle += 2 * pi
        d_dist = dist_two - dist_one
        dx = d_dist * cos(pi / 2 - angle_one)
        dy = d_dist * sin(pi / 2 - angle_one)
        if abs(dx) > 0.5 or abs(dy) > 0.5 or abs(d_angle) > pi / 2:
            continue

        avg_d_angle += d_angle
        avg_dx += dx
        avg_dy += dy
        ok_pairs_count += 1

    if ok_pairs_count == 0:
        return None
    avg_d_angle /= ok_pairs_count
    avg_dx /= ok_pairs_count
    avg_dy /= ok_pairs_count

    assert abs(avg_dx) < 0.5 and abs(avg_dy) < 0.5 and abs(avg_d_angle) < pi / 2, \
        'INVALID ERR: dx={:+0.04f}, dy={:+0.04f}, dteta={:+0.04f}'.format(avg_dx, avg_dy, avg_d_angle)

    return avg_dx, avg_dy, avg_d_angle
