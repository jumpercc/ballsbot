from math import sqrt, sin, cos

from ballsbot.lidar import apply_transformation_to_cloud
from ballsbot.geometry import get_linear_coefs, distance, point_to_line_distance, normal_to_line_in_point, \
    get_45_degrees_lines

VOXEL_SIZE = 0.1
VOXELS_PER_CELL = 10
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

    transformation = [0., 0., pose['teta']]
    points = apply_transformation_to_cloud(points, transformation)

    sqare_part = 0.18 / (CELL_SIZE * CELL_SIZE)
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
            if voxels_count > 0 and float(free_voxels) / voxels_count >= sqare_part:
                result[cell_y][cell_x] = True

    _filter_disconnected_cells(result)

    half_size_in_cells = int(size_in_cells / 2.)
    return result, [int(round(pose['x'] / CELL_SIZE)) - half_size_in_cells,
                    int(round(pose['y'] / CELL_SIZE)) - half_size_in_cells]


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
    # inside_a_cell = distance([(cell_x + 0.5) * CELL_SIZE, (cell_y + 0.5) * CELL_SIZE], [x, y]) <= CELL_SIZE / 2.
    return cell_x, cell_y


class Grid:
    SEEN_MEMORY = 100  # 50 -> 12.5 sec when 4 fps
    WAS_IN_MEMORY = 200
    MIN_SEEN = 0

    def __init__(self):
        self.grid = {}
        self.counter = 0

    def update_grid(self, points, pose):
        cells, shift_in_cells = cloud_to_free_cells(points, pose)
        shift_x, shift_y = shift_in_cells
        for cell_y in range(len(cells)):
            for cell_x in range(len(cells)):
                grid_key = (cell_x + shift_x, cell_y + shift_y)
                if cells[cell_y][cell_x]:
                    if grid_key not in self.grid:
                        self.grid[grid_key] = {
                            'seen': 0,
                        }
                    self.grid[grid_key]['seen_at'] = self.counter
                    self.grid[grid_key]['seen'] += 1
                elif grid_key in self.grid and 'was_at' not in self.grid[grid_key] and self.grid[grid_key]['seen'] < 6:
                    self.grid[grid_key]['seen'] -= 1

        car_x, car_y = get_car_cell(pose['x'], pose['y'])
        grid_key = (car_x, car_y)
        if grid_key not in self.grid:
            self.grid[grid_key] = {
                'seen_at': self.counter,
                'seen': 0,
            }
        self.grid[grid_key]['was_at'] = self.counter

        for grid_key, value in list(self.grid.items()):
            if 'was_at' in value:
                if value['was_at'] + self.WAS_IN_MEMORY < self.counter:
                    del self.grid[grid_key]
            elif value['seen_at'] + self.SEEN_MEMORY < self.counter:
                del self.grid[grid_key]
            elif value['seen'] <= self.MIN_SEEN:
                del self.grid[grid_key]

        self.counter += 1

    def get_cell_weight(self, x, y):
        grid_key = (x, y)
        if grid_key in self.grid:
            cell = self.grid[grid_key]
            if 'was_at' in cell:
                return (self.counter - cell['was_at']) / (2. * self.SEEN_MEMORY)
            else:
                return 1.
        else:
            return 0.

    def get_directions_weights(self, pose, car_info):
        """
        :param pose: {'x': ..., 'y': ..., 'teta': ...}
        :param car_info: car sizes {'to_car_center': ..., 'turn_radius': ...}
        :return: weights {(turn, direction): weight, ...}
        """
        result = {
            (-1., 1.): 0.,
            (-0.5, 1.): 0.,
            (0., 1.): 0.,
            (0.5, 1.): 0.,
            (1., 1.): 0.,
            (-1., -1.): 0.,
            (-0.5, -1.): 0.,
            (0., -1.): 0.,
            (0.5, -1.): 0.,
            (1., -1.): 0.,
        }

        shift = car_info['to_car_center'] + car_info['turn_radius'] / 2.
        front_point = [
            pose['x'] + shift * cos(pose['teta']),
            pose['y'] + shift * sin(pose['teta']),
        ]
        shift = car_info['to_car_center'] - car_info['turn_radius'] / 2.
        rear_point = [
            pose['x'] + shift * cos(pose['teta']),
            pose['y'] + shift * sin(pose['teta']),
        ]

        central_line = get_linear_coefs(front_point, rear_point)
        front_normal = normal_to_line_in_point(central_line, front_point)
        rear_normal = normal_to_line_in_point(central_line, rear_point)
        front_45s = get_45_degrees_lines(central_line, front_point)
        rear_45s = get_45_degrees_lines(central_line, rear_point)

        # print('pose {}, {}, {}'.format(pose['x'], pose['y'], pose['teta']))
        # print('points {}, {}'.format(front_point, rear_point))
        # print('central line: {}'.format(central_line))
        # print('normals {}, {}'.format(front_normal, rear_normal))
        # print('45s {}, {}'.format(front_45s, rear_45s))

        # TODO перебираем доступные клетки, считаем, в какой из 10ти секторов он попадает
        for grid_key, cell_info in self.grid.items():
            pass  # TODO зависит от наличия и значения was_at

        return result
