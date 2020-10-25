from math import sqrt

from ballsbot.lidar import apply_transformation_to_cloud
from ballsbot.geometry import get_linear_coefs, distance, point_to_line_distance

VOXEL_SIZE = 0.1
VOXELS_PER_CELL = 6
CELL_SIZE = VOXELS_PER_CELL * VOXEL_SIZE


def cloud_to_voxels(points, half_width=8.):
    size_in_voxels = 2 * int(round(half_width / VOXEL_SIZE))
    result = [[0 for _ in range(size_in_voxels)] for _ in range(size_in_voxels)]
    for a_point in points:
        cell_x = int(round((a_point[0] + half_width) / VOXEL_SIZE))
        if cell_x < 0:
            cell_x = 0
        elif cell_x >= size_in_voxels:
            cell_x = size_in_voxels - 1

        cell_y = int(round((a_point[1] + half_width) / VOXEL_SIZE))
        if cell_y < 0:
            cell_y = 0
        elif cell_y >= size_in_voxels:
            cell_y = size_in_voxels - 1

        result[cell_y][cell_x] += 1

    center_shift = int(size_in_voxels / 2)
    for i in range(2):
        for j in range(2):
            _mark_hidden_voxels(result, center_shift + i - 1, center_shift + j - 1, center_shift)

    return result


def _mark_hidden_voxels(voxels, start_x, start_y, center_shift):
    queue = [[start_x, start_y]]
    seen = set()
    lines = []
    car_position = [float(center_shift), float(center_shift)]
    w_max = sqrt(2 * 0.5 * 0.5)

    i = 0
    while i < len(queue):
        x, y = queue[i]
        i += 1

        if (x, y) in seen or x < 0 or y < 0 or x >= len(voxels) or y >= len(voxels):
            continue
        seen.add((x, y))
        cell_center = [x + 0.5, y + 0.5]

        if voxels[y][x] > 0:
            a, b, c = get_linear_coefs(car_position, cell_center)
            d = distance(car_position, cell_center)
            lines.append([a, b, c, d])
        else:
            D = distance(car_position, cell_center)
            for a_line in lines:
                a, b, c, d = a_line
                W = point_to_line_distance(cell_center, a, b, c)
                if W * d / D <= w_max:
                    voxels[y][x] = -1
                    break

        if x < center_shift:
            if y < center_shift:  # left down
                queue.append([x - 1, y])
                queue.append([x - 1, y - 1])
                queue.append([x, y - 1])
            else:  # left up
                queue.append([x - 1, y])
                queue.append([x - 1, y + 1])
                queue.append([x, y + 1])
        else:
            if y < center_shift:  # right down
                queue.append([x + 1, y])
                queue.append([x + 1, y - 1])
                queue.append([x, y - 1])
            else:  # right up
                queue.append([x + 1, y])
                queue.append([x + 1, y + 1])
                queue.append([x, y + 1])


def cloud_to_free_cells(points, pose, half_width=4. * CELL_SIZE):
    size_in_cells = 2 * int(round((half_width / CELL_SIZE)))
    result = [[False for _ in range(size_in_cells)] for _ in range(size_in_cells)]

    transformation = [-(pose['x'] % CELL_SIZE), -(pose['y'] % CELL_SIZE), -pose['teta']]
    points = apply_transformation_to_cloud(points, transformation)

    voxels = cloud_to_voxels(points, half_width)
    for cell_y in range(size_in_cells):
        for cell_x in range(size_in_cells):
            voxels_count = 0
            free_voxels = 0
            for voxel_y in range(VOXELS_PER_CELL):
                voxel_y += cell_y * VOXELS_PER_CELL
                if voxel_y >= len(voxels):
                    break
                for voxel_x in range(VOXELS_PER_CELL):
                    voxel_x += cell_x * VOXELS_PER_CELL
                    if voxel_x >= len(voxels):
                        break
                    voxels_count += 1
                    if voxels[voxel_y][voxel_x] == 0:
                        free_voxels += 1
            if voxels_count > 0 and float(free_voxels) / voxels_count >= 0.5:
                result[cell_y][cell_x] = True

    _filter_disconnected_cells(result)

    return result


def _filter_disconnected_cells(cells):
    queue = []

    center_shift = int(len(cells) / 2)
    for i in range(2):
        for j in range(2):
            queue.append([center_shift + i - 1, center_shift + j - 1])

    seen = set()

    i = 0
    while i < len(queue):
        x, y = queue[i]
        i += 1

        if (x, y) in seen or x < 0 or y < 0 or x >= len(cells) or y >= len(cells):
            continue
        seen.add((x, y))

        if x < center_shift:
            if y < center_shift:  # left down
                if cells[y][x] and not (cells[y + 1][x] or cells[y + 1][x + 1] or cells[y][x + 1]):
                    cells[y][x] = False
                queue.append([x - 1, y])
                queue.append([x - 1, y - 1])
                queue.append([x, y - 1])
            else:  # left up
                if cells[y][x] and not (cells[y - 1][x] or cells[y - 1][x + 1] or cells[y][x + 1]):
                    cells[y][x] = False
                queue.append([x - 1, y])
                queue.append([x - 1, y + 1])
                queue.append([x, y + 1])
        else:
            if y < center_shift:  # right down
                if cells[y][x] and not (cells[y + 1][x] or cells[y + 1][x - 1] or cells[y][x - 1]):
                    cells[y][x] = False
                queue.append([x + 1, y])
                queue.append([x + 1, y - 1])
                queue.append([x, y - 1])
            else:  # right up
                if cells[y][x] and not (cells[y - 1][x] or cells[y - 1][x - 1] or cells[y][x - 1]):
                    cells[y][x] = False
                queue.append([x + 1, y])
                queue.append([x + 1, y + 1])
                queue.append([x, y + 1])


def get_car_cell(x, y):
    cell_x = int(round((x / CELL_SIZE)))
    cell_y = int(round((y / CELL_SIZE)))
    inside_a_cell = distance([cell_x, cell_y], [x, y]) <= CELL_SIZE / 2.
    return cell_x, cell_y, inside_a_cell
