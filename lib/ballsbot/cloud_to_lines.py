from math import sqrt
import numpy as np

MIN_CLUSTER_SIZE = 10
MAX_TRASH_SIZE = 5
MAX_CLUSTER_HOLE = 0.1
WALL_MAX_DISTANCE = 0.1
WALL_TRASH_SIZE = 10
MAX_WALL_GAP = 0.5
WALL_MAX_DISTANCE_2ND = 0.03
MIN_LINE_LENGTH = 0.5


def distance(one, two):
    return sqrt((one[0] - two[0]) * (one[0] - two[0]) + (one[1] - two[1]) * (one[1] - two[1]))


def avg_point(points):
    points = np.array(points)
    result = np.median(np.array(points), axis=0)
    return result[0], result[1]


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
    result = []

    start = 0
    end = start + MIN_CLUSTER_SIZE - 1
    while start < len(point_numbers) - 1:
        part_points = [all_points[x] for x in point_numbers[start:end + 1]]
        a, b, c = get_linear_coefs(part_points)

        a_wall = []
        last_index = None
        for local_index, point_number in enumerate(point_numbers[start:]):
            p0 = all_points[point_number]
            d = point_to_line_distance(p0, a, b, c)
            if last_index is not None and distance(p0, all_points[point_numbers[start + last_index]]) > MAX_WALL_GAP:
                break
            elif d <= WALL_MAX_DISTANCE:
                a_wall.append(point_number)
                last_index = local_index
            elif last_index is not None and local_index - last_index > WALL_TRASH_SIZE:
                part_points = [all_points[x] for x in point_numbers[start:start + last_index]]
                an, bn, cn = get_linear_coefs(part_points)
                d = point_to_line_distance(p0, a, b, c)
                if d <= WALL_MAX_DISTANCE / 2.:
                    a_wall.append(point_number)
                    last_index = local_index
                    a, b, c = an, bn, cn
                else:
                    break
        if len(a_wall):
            if len(a_wall) >= MIN_CLUSTER_SIZE:
                result.append(a_wall)
            start = start + last_index + 1
            end = start + MIN_CLUSTER_SIZE - 1
            if end > len(point_numbers) - 1:
                end = len(point_numbers) - 1

    return result


def get_cnumber_by_index(clusters, pose_points):
    cnumber_by_index = {}
    merge_clusters = (len(clusters) > 1) and (
            distance(pose_points[clusters[0][0]], pose_points[clusters[-1][-1]]) <= MAX_CLUSTER_HOLE)
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
            if i - end > MAX_TRASH_SIZE:
                break
            if distance(pose_points[end], pose_points[i]) <= MAX_CLUSTER_HOLE:
                end = i
                cluster_points.append(i)
        if end - start >= MIN_CLUSTER_SIZE:
            if walls and (end - start >= 2 * MIN_CLUSTER_SIZE):
                cluster_points_list = split_walls(cluster_points, pose_points)
            else:
                cluster_points_list = [cluster_points]
            clusters += cluster_points_list
        start = end + 1
    return clusters


def cnumber_by_index_to_clusters(cnumber_by_index):
    clusters = {}
    for i, cluster_number in cnumber_by_index.items():
        if cluster_number not in clusters:
            clusters[cluster_number] = []
        clusters[cluster_number].append(i)
    return list(sorted([list(sorted(x)) for x in clusters.values()], key=lambda y: y[0]))


def flatten_a_list(in_struct):
    return [in_struct[0]] + in_struct[1]


def recalculate_walls(clusters, all_points):
    result = {}
    for i in sum(clusters, []):
        result[i] = {}

    for cluster_number, point_numbers in enumerate(clusters):
        start = len(point_numbers) // 3
        part_points = [all_points[x] for x in point_numbers[start:2 * start + 1]]
        a, b, c = get_linear_coefs(part_points)

        if len(clusters) == 1:
            all_point_numbers = point_numbers
        elif len(clusters) == 2:
            all_point_numbers = clusters[0] + clusters[1]
        elif cluster_number == 0:
            all_point_numbers = clusters[-1] + point_numbers + clusters[cluster_number + 1]
        elif cluster_number == len(clusters) - 1:
            all_point_numbers = clusters[cluster_number - 1] + point_numbers + clusters[0]
        else:
            all_point_numbers = clusters[cluster_number - 1] + point_numbers + clusters[cluster_number + 1]
        all_point_numbers = list(sorted(all_point_numbers))

        selected = []
        for point_number in all_point_numbers:
            d = point_to_line_distance(all_points[point_number], a, b, c)
            if d <= WALL_MAX_DISTANCE_2ND:
                selected.append([point_number, d])

        start = 0
        end = 0
        sub_clusters = []
        for i, it in enumerate(selected):
            if i == start:
                continue
            point_number, _ = it
            if distance(all_points[point_number], all_points[selected[i - 1][0]]) <= MAX_WALL_GAP:
                end = i
            else:
                if end - start + 1 >= MIN_CLUSTER_SIZE:
                    sub_clusters.append(selected[start:end + 1])
                start = end = i
        if end - start + 1 >= MIN_CLUSTER_SIZE:
            sub_clusters.append(selected[start:end + 1])

        if len(sub_clusters) == 1:
            selected = sub_clusters[0]
            for point_number, d in selected:
                result[point_number][cluster_number] = [d, a, b, c]
        elif len(sub_clusters) > 0:
            for i, selected in enumerate(sub_clusters):
                local_cluster_number = 1000 * cluster_number + i
                for point_number, d in selected:
                    result[point_number][local_cluster_number] = [d, a, b, c]

    agg_result = {
        y[0]: flatten_a_list(sorted(
            y[1].items(),
            key=lambda z: z[1][0]
        )[0]) for y in filter(
            lambda x: len(x[1]) > 0, result.items()
        )
    }

    return cnumber_by_index_to_clusters({x[0]: x[1][0] for x in agg_result.items()}), \
           {x[0]: x[1][2:] for x in agg_result.items()}


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
            if point_to_line_distance(p, a, b, c) > WALL_MAX_DISTANCE_2ND:
                continue
            min_x = p[0]
            corners[0] = i
        if p[0] > max_x:
            if point_to_line_distance(p, a, b, c) > WALL_MAX_DISTANCE_2ND:
                continue
            max_x = p[0]
            corners[1] = i
        if p[1] < min_y:
            if point_to_line_distance(p, a, b, c) > WALL_MAX_DISTANCE_2ND:
                continue
            min_y = p[1]
            corners[2] = i
        if p[1] > max_y:
            if point_to_line_distance(p, a, b, c) > WALL_MAX_DISTANCE_2ND:
                continue
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


def cloud_to_lines(pose_points):
    clusters = clusterize_cloud(pose_points, walls=False)
    filtered_point_indexes = sum(clusters, [])
    clusters = split_walls(filtered_point_indexes, pose_points)
    clusters, abc_by_index = recalculate_walls(clusters, pose_points)

    lines = []
    for a_cluster in sorted(clusters, key=lambda x: x[0]):
        if len(a_cluster) < MIN_CLUSTER_SIZE:
            continue
        a, b, c = abc_by_index[a_cluster[0]]
        ends = cluster_to_line([pose_points[x] for x in a_cluster], a, b, c)
        if distance(ends[0], ends[1]) >= MIN_LINE_LENGTH:
            lines.append(ends)

    return lines
