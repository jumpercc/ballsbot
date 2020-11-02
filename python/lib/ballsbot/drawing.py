from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
from io import BytesIO

from ballsbot.utils import figsize_from_image_size


def update_image_abs_coords(
        image, poses, lidar_points, self_position, only_nearby_meters, figsize=None,
        tail_points=None, tail_lines=None, lines=None
):
    if figsize is None:
        figsize = figsize_from_image_size(image)
    poses_x_points = [x['x'] for x in poses]
    poses_y_points = [x['y'] for x in poses]
    if len(poses) == 0:
        pose = {'x': 0., 'y': 0., 'teta': 0.}
    else:
        pose = poses[-1]

    fig = Figure(figsize=figsize)
    canvas = FigureCanvas(fig)
    ax = fig.gca()

    ax.set_xlim(-only_nearby_meters, only_nearby_meters)
    ax.set_ylim(-only_nearby_meters, only_nearby_meters)

    ax.scatter(poses_x_points, poses_y_points, marker='o', s=1, c='gray')

    if tail_points:
        tail_x_points = [x[0] for x in tail_points]
        tail_y_points = [x[1] for x in tail_points]
        ax.scatter(tail_x_points, tail_y_points, marker='o', s=5, c='lightblue')
    if tail_lines:
        for a_line in tail_lines:
            x_points = [a_line[0][0], a_line[1][0]]
            y_points = [a_line[0][1], a_line[1][1]]
            ax.plot(x_points, y_points, c='lightblue')

    lidar_x_points = [x[0] for x in lidar_points]
    lidar_y_points = [x[1] for x in lidar_points]
    ax.scatter(lidar_x_points, lidar_y_points, marker='o', s=5, c='blue')
    if lines:
        for a_line in lines:
            x_points = [a_line[0][0], a_line[1][0]]
            y_points = [a_line[0][1], a_line[1][1]]
            ax.plot(x_points, y_points, c='orange', linewidth=3)

    if self_position is not None:
        rect = patches.Rectangle(
            (self_position['x'] + pose['x'], self_position['y'] + pose['y']), self_position['w'], self_position['h'],
            linewidth=3, edgecolor='r', facecolor='none'
        )

        rotation_center = ax.transData.transform([
            self_position['x'] + pose['x'] + self_position['w'] / 2,
            self_position['y'] + pose['y'] + self_position['h'] / 2,
        ])
        rotation = Affine2D().rotate_around(
            rotation_center[0],
            rotation_center[1],
            pose['teta']
        )
        rect.set_transform(ax.transData + rotation)
        ax.add_patch(rect)
    else:
        ax.scatter([pose['x']], [pose['y']], marker='o', s=50, c='r')

    ax.text(
        -only_nearby_meters, -only_nearby_meters,
        'x: {:0.02f}, y:{:0.02f}, teta: {:0.02f}, pose {}'.format(pose['x'], pose['y'], pose['teta'], len(poses)),
        fontsize=10
    )

    ax.grid(which='both', linestyle='--', alpha=0.5)

    canvas.draw()
    jpeg = BytesIO()
    canvas.print_jpg(jpeg)
    image.value = jpeg.getvalue()


def update_picture_self_coords(image, lidar_points, self_position, only_nearby_meters, additional_points):
    fig = Figure(figsize=figsize_from_image_size(image))
    canvas = FigureCanvas(fig)
    ax = fig.gca()

    x_points = [x[0] for x in lidar_points]
    y_points = [x[1] for x in lidar_points]
    ax.scatter(x_points, y_points, marker='o', s=5, c='b')

    if self_position is None:
        ax.scatter([0], [0], marker='o', s=50, c='r')
    else:
        rect = patches.Rectangle(
            (self_position['x'], self_position['y']), self_position['w'], self_position['h'],
            linewidth=3, edgecolor='r', facecolor='none'
        )
        ax.add_patch(rect)

    if additional_points is not None:
        ax.scatter(additional_points['x'], additional_points['y'], marker='o', s=25, c='g')

    ax.set_xlim(-only_nearby_meters, only_nearby_meters)
    ax.set_ylim(-only_nearby_meters, only_nearby_meters)
    ax.grid(which='both', linestyle='--', alpha=0.5)

    canvas.draw()
    jpeg = BytesIO()
    canvas.print_jpg(jpeg)
    image.value = jpeg.getvalue()
