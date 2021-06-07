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


class ManipulatorPoseDrawing:
    def __init__(self, widgets):
        self.projection_xy = widgets.Image(format='jpeg', width=400, height=400)
        self.projection_yz = widgets.Image(format='jpeg', width=400, height=400)
        self.projection_xz = widgets.Image(format='jpeg', width=400, height=400)
        self.stub_image = widgets.Image(format='jpeg', width=400, height=400)
        row1 = widgets.HBox([self.projection_xy, self.stub_image])
        row2 = widgets.HBox([self.projection_xz, self.projection_yz])
        self.images_grid = widgets.VBox([row1, row2])

    def get_images(self):
        return self.images_grid

    def update_images(self, manipulator_pose, override_crop_half_size=None):
        if override_crop_half_size:
            crop_half_size = override_crop_half_size
        else:
            crop_half_size = 1.

        x_points = []
        y_points = []
        z_points = []
        for a_point in manipulator_pose['points']:
            if not override_crop_half_size:
                crop_half_size = max(crop_half_size, *a_point)
            x, y, z = a_point
            x_points.append(x)
            y_points.append(y)
            z_points.append(z)

        if manipulator_pose.get('claw_points'):
            claw_points = (
                manipulator_pose['points'][-1], manipulator_pose['claw_points'][0],
                manipulator_pose['points'][-1], manipulator_pose['claw_points'][1],
            )
            for a_point in claw_points:
                if not override_crop_half_size:
                    crop_half_size = max(crop_half_size, *a_point)
                x, y, z = a_point
                x_points.append(x)
                y_points.append(y)
                z_points.append(z)

        if not override_crop_half_size:
            crop_half_size *= 1.05

        color = [
            'orange',
            'black',
            'red',
            'green',
            'blue',
            'blue',
            'blue',
            'blue',
        ]

        images = {'xy': self.projection_xy, 'xz': self.projection_xz, 'yz': self.projection_yz, '': self.stub_image}
        for image_type, image in images.items():
            fig = Figure(figsize=figsize_from_image_size(image))
            canvas = FigureCanvas(fig)
            ax = fig.gca()

            ax.text(-crop_half_size, -crop_half_size, image_type, fontsize=16)

            if image_type == 'xy':
                first_points = x_points
                second_points = y_points
            elif image_type == 'xz':
                first_points = x_points
                second_points = z_points
            elif image_type == 'yz':
                first_points = y_points
                second_points = z_points
            else:
                first_points = []
                second_points = []
                ax.text(
                    -crop_half_size, -crop_half_size,
                    self.format_pose(manipulator_pose),
                    fontsize=10, color='green'
                )

            prev_first = 0.
            prev_second = 0.
            for i in range(len(first_points)):
                ax.plot(
                    [prev_first, first_points[i]],
                    [prev_second, second_points[i]],
                    c=color[i], linewidth=3
                )
                prev_first = first_points[i]
                prev_second = second_points[i]

            ax.scatter(first_points, second_points, marker='o', s=15, c='b')

            ax.set_xlim(-crop_half_size, crop_half_size)
            ax.set_ylim(-crop_half_size, crop_half_size)
            if image_type:
                ax.grid(which='both', linestyle='--', alpha=0.5)
            else:
                ax.axis('off')

            canvas.draw()
            jpeg = BytesIO()
            canvas.print_jpg(jpeg)
            image.value = jpeg.getvalue()

    @staticmethod
    def format_pose(pose):
        result = 'rotations:\n'
        for it in pose['rotations']:
            result += '    ({:0.02f}, {:0.02f}, {:0.02f})\n'.format(*it)

        result += 'points:\n'
        for it in pose['points']:
            result += '    ({:0.0f}, {:0.0f}, {:0.0f})\n'.format(*it)

        result += 'claw_points:\n'
        for it in pose['claw_points']:
            result += '    ({:0.0f}, {:0.0f}, {:0.0f})\n'.format(*it)

        return result
