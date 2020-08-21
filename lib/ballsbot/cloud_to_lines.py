from math import sqrt

min_cluster_size = 10
max_trash_size = 5
max_cluster_hole = 0.1
wall_max_distance = 0.15
wall_trash_size = 10
max_wall_gap = 0.5
wall_max_distance_2nd = 0.03
min_line_length = 0.3


def distance(one, two):
    return sqrt((one[0] - two[0]) * (one[0] - two[0]) + (one[1] - two[1]) * (one[1] - two[1]))


def avg_point(points):
    sum_x = 0.
    sum_y = 0.
    for p in points:
        sum_x += p[0]
        sum_y += p[1]
    return [sum_x / len(points), sum_y / len(points)]


def get_linear_coefs(part_points):
    p1 = avg_point(part_points[0:len(part_points) // 2])
    p2 = avg_point(part_points[len(part_points) // 2:])
    a = p1[1] - p2[1]
    b = p2[0] - p1[0]
    c = p1[0] * p2[1] - p2[0] * p1[1]
    return a, b, c


def point_to_line_distance(p0, a, b, c):
    return abs(a * p0[0] + b * p0[1] + c) / sqrt(a * a + b * b)


def split_walls(point_numbers, all_points):
    # print('SPLIT {}-{}'.format(point_numbers[0], point_numbers[-1]))
    result = []

    start = 0
    end = start + min_cluster_size - 1
    while start < len(point_numbers) - 1:
        # print('start {} - {} / {}'.format(start, end, len(point_numbers)))
        part_points = [all_points[x] for x in point_numbers[start:end + 1]]
        a, b, c = get_linear_coefs(part_points)

        a_wall = []
        last_index = None
        for local_index, point_number in enumerate(point_numbers[start:]):
            p0 = all_points[point_number]
            d = point_to_line_distance(p0, a, b, c)
            if last_index is not None and distance(p0, all_points[point_numbers[start + last_index]]) > max_wall_gap:
                # print('too long {}, ({},{})'.format(point_number, start, len(point_numbers) - 1))
                break
            elif d <= wall_max_distance:
                # print('ok {}'.format(point_number))
                a_wall.append(point_number)
                last_index = local_index
            elif last_index is not None and local_index - last_index > wall_trash_size:
                old_d = d
                # print('recalculating from [{}, {}) to [{}, {})'.format(start, end + 1, start, start + last_index))
                part_points = [all_points[x] for x in point_numbers[start:start + last_index]]
                an, bn, cn = get_linear_coefs(part_points)
                d = point_to_line_distance(p0, a, b, c)
                if d <= wall_max_distance / 2.:
                    # print('ok {} after recalculation ({:0.3f} -> {:0.3f})'.format(point_number, old_d, d))
                    a_wall.append(point_number)
                    last_index = local_index
                    a, b, c = an, bn, cn
                else:
                    # print('skip {}'.format(point_number))
                    break
            # else:
            #     print('trash {}'.format(point_number))
        if len(a_wall):
            # print(a_wall)
            if len(a_wall) >= min_cluster_size:
                result.append(a_wall)
            # else:
            #     print('TRASH {}'.format(len(a_wall)))
            start = start + last_index + 1
            end = start + min_cluster_size - 1
            if end > len(point_numbers) - 1:
                end = len(point_numbers) - 1
        # print('end {} - {} / {}'.format(start, end, len(point_numbers)))

    return result


def get_cnumber_by_index(clusters, pose_points):
    cnumber_by_index = {}
    merge_clusters = (len(clusters) > 1) and (
            distance(pose_points[clusters[0][0]], pose_points[clusters[-1][-1]]) <= max_cluster_hole)
    for cnumber, cluster_points in enumerate(clusters):
        if merge_clusters and cnumber == len(clusters) - 1:
            cnumber = 0
        for p in cluster_points:
            cnumber_by_index[p] = cnumber

    return cnumber_by_index


def clusterize_cloud(pose_points, walls=False):
    clusters = []
    start = 0
    while start < len(pose_points):
        cluster_points = [start]
        end = start
        for i in range(end + 1, len(pose_points)):
            if i - end > max_trash_size:
                break
            if distance(pose_points[end], pose_points[i]) <= max_cluster_hole:
                end = i
                cluster_points.append(i)
        if end - start >= min_cluster_size:
            if walls and (end - start >= 2 * min_cluster_size):
                cluster_points_list = split_walls(cluster_points, pose_points)
            else:
                cluster_points_list = [cluster_points]
            clusters += cluster_points_list
        start = end + 1  # TODO intersection
    return get_cnumber_by_index(clusters, pose_points)


def cnumber_by_index_to_clusters(cnumber_by_index):
    clusters = {}  # TODO optimize
    for i, cluster_number in cnumber_by_index.items():
        if cluster_number not in clusters:
            clusters[cluster_number] = []
        clusters[cluster_number].append(i)
    return list(sorted([list(sorted(x)) for x in clusters.values()], key=lambda y: y[0]))


def flatten_a_list(in_struct):
    return [in_struct[0]] + in_struct[1]


def recalculate_walls(cnumber_by_index, all_points):
    result = {}
    clusters = cnumber_by_index_to_clusters(cnumber_by_index)
    for i in cnumber_by_index.keys():
        result[i] = {}

    for cluster_number, point_numbers in enumerate(clusters):
        start = len(point_numbers) // 3
        part_points = [all_points[x] for x in point_numbers[start:2 * start + 1]]
        a, b, c = get_linear_coefs(part_points)

        if len(clusters) == 1:
            all_point_numbers = point_numbers
        if len(clusters) == 2:
            all_point_numbers = clusters[0] + clusters[1]
        elif cluster_number == 0:
            all_point_numbers = clusters[-1] + point_numbers + clusters[cluster_number + 1]
        elif cluster_number == len(clusters) - 1:
            all_point_numbers = clusters[cluster_number - 1] + point_numbers + clusters[0]
        else:
            all_point_numbers = clusters[cluster_number - 1] + point_numbers + clusters[cluster_number + 1]

        for point_number in all_point_numbers:
            d = point_to_line_distance(all_points[point_number], a, b, c)
            if d <= wall_max_distance_2nd:  # TODO max_wall_gap
                result[point_number][cluster_number] = [d, a, b, c]

    agg_result = {
        y[0]: flatten_a_list(sorted(
            y[1].items(),
            key=lambda z: z[1][0]
        )[0]) for y in filter(
            lambda x: len(x[1]) > 0, result.items()
        )
    }

    return {x[0]: x[1][0] for x in agg_result.items()}, {x[0]: x[1][2:] for x in agg_result.items()}


def point_on_line_projection(p0, a, b, c):
    i = - (a * p0[0] + b * p0[1] + c) / (a * a + b * b)
    y1 = p0[1] + b * i
    x1 = p0[0] + a * i
    return [x1, y1]


def cluster_to_line(points, a, b, c):
    assert len(points) > 1, points
    max_y = max_x = -1e10
    min_y = min_x = 1e10
    corners = [None, None, None, None]
    for i, p in enumerate(points):
        if p[0] < min_x:
            min_x = p[0]
            corners[0] = i
        if p[0] > max_x:
            max_x = p[0]
            corners[1] = i
        if p[1] < min_y:
            min_y = p[1]
            corners[2] = i
        if p[1] > max_y:
            max_y = p[1]
            corners[3] = i
    corners = list(set(corners))

    projections = [point_on_line_projection(points[x], a, b, c) for x in corners]
    assert len(projections) >= 2, corners
    if len(projections) == 2:
        ends = projections
    else:
        max_d = -1e10
        ends = None
        for i in range(len(projections)):
            for j in range(i + 1, len(projections)):
                d = distance(projections[i], projections[j])
                if d > max_d:
                    max_d = d
                    ends = [projections[i], projections[j]]

    one, two = ends
    if abs(one[0] - two[0]) > abs(one[1] - two[1]):
        sort_index = 0
    else:
        sort_index = 1
    return list(sorted(ends, key=lambda x: x[sort_index])) + [a, b, c]


def cloud_to_lines(pose_points):  # TODO optimize
    cnumber_by_index = clusterize_cloud(pose_points, walls=False)

    cnumber_by_index = get_cnumber_by_index(
        split_walls(
            list(cnumber_by_index.keys()),
            pose_points
        ),
        pose_points
    )
    cnumber_by_index, abc_by_index = recalculate_walls(cnumber_by_index, pose_points)

    clusters = cnumber_by_index_to_clusters(cnumber_by_index)

    lines = []
    for a_cluster in clusters:
        if len(a_cluster) < min_cluster_size:
            continue
        a, b, c = abc_by_index[a_cluster[0]]
        ends = cluster_to_line([pose_points[x] for x in a_cluster], a, b, c)
        if distance(ends[0], ends[1]) >= min_line_length:
            lines.append(ends)

    return lines
