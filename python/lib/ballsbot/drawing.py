from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib import patches
from matplotlib.transforms import Affine2D
from io import BytesIO

from ballsbot.utils import figsize_from_image_size
from ballsbot.config import LIDAR_CALIBRATION_WITHOUT_MANIPULATOR, ENABLE_MULTIPROCESSING


class BotPoseDrawing:
    def __init__(self, image):
        self.image = image

    def update_image(self, poses, lidar_points, self_position, free_cells=None, target_point=None,  # pylint: disable=R0913
                     image_text=None, only_nearby_meters=6.):
        figsize = figsize_from_image_size(self.image)
        params = [
            poses, lidar_points, self_position, only_nearby_meters, figsize, free_cells, target_point, image_text]

        if ENABLE_MULTIPROCESSING:
            drawer = DrawInAnotherProcess.get_instance('get_manipulator_drawing_images')
            value = drawer.draw('get_image_abs_coords', *params)
        else:
            value = get_image_abs_coords(*params)

        self.image.value = value


def get_image_abs_coords(
        poses, lidar_points, self_position, only_nearby_meters, figsize,
        free_cells=None, target_point=None, image_text=None
):  # pylint: disable=R0913, R0914
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

    if free_cells:
        tail_x_points = [x[0] for x in free_cells]
        tail_y_points = [x[1] for x in free_cells]
        ax.scatter(tail_x_points, tail_y_points, marker='o', s=5, c='lightblue')
    if target_point:
        target_x_points = [target_point[0]]
        target_y_points = [target_point[1]]
        ax.scatter(target_x_points, target_y_points, marker='x', s=5, c='red')

    lidar_x_points = [x[0] for x in lidar_points]
    lidar_y_points = [x[1] for x in lidar_points]
    ax.scatter(lidar_x_points, lidar_y_points, marker='o', s=5, c='blue')

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

    if image_text is not None:
        ax.text(-only_nearby_meters, -only_nearby_meters, image_text, fontsize=10)

    ax.grid(which='both', linestyle='--', alpha=0.5)

    fig.tight_layout()
    canvas.draw()
    jpeg = BytesIO()
    canvas.print_jpg(jpeg)
    return jpeg.getvalue()


def update_image_abs_coords(image, *args, **kwargs):
    image.value = get_image_abs_coords(*args, **kwargs)


def update_picture_self_coords(image, lidar_points, self_position, only_nearby_meters, additional_points):
    figsize = figsize_from_image_size(image)
    params = [figsize, lidar_points, self_position, only_nearby_meters, additional_points]
    if ENABLE_MULTIPROCESSING:
        drawer = DrawInAnotherProcess.get_instance('get_picture_self_coords')
        value = drawer.draw('get_picture_self_coords', *params)
    else:
        value = get_picture_self_coords(*params)
    image.value = value


def get_picture_self_coords(figsize, lidar_points, self_position, only_nearby_meters, additional_points):
    fig = Figure(figsize=figsize)
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
    return jpeg.getvalue()


class ManipulatorPoseDrawing:
    def __init__(self, widgets):
        self.projection_xy = widgets.Image(format='jpeg', width=400, height=400)
        self.projection_xz = widgets.Image(format='jpeg', width=400, height=400)
        self.images_grid = widgets.HBox([self.projection_xy, self.projection_xz])

    def get_images(self):
        return self.images_grid

    def update_images(self, manipulator_pose, override_crop_half_size=None):
        images = {'xy': self.projection_xy, 'xz': self.projection_xz}
        figsizes = {}
        for image_type, image in images.items():
            figsizes[image_type] = figsize_from_image_size(image)
        params = [figsizes, manipulator_pose, override_crop_half_size]

        if ENABLE_MULTIPROCESSING:
            drawer = DrawInAnotherProcess.get_instance('get_manipulator_drawing_images')
            values = drawer.draw('get_manipulator_drawing_images', *params)
        else:
            raise NotImplementedError()

        for image_type, value in values.items():
            images[image_type].value = value


# pylint: disable=E1130, R0914, R0915
def get_manipulator_drawing_images(figsizes, manipulator_pose, override_crop_half_size=None):
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

    result = {}
    for image_type, figsize in figsizes.items():
        fig = Figure(figsize=figsize)
        canvas = FigureCanvas(fig)
        ax = fig.gca()

        ax.text(-crop_half_size, -crop_half_size, image_type, fontsize=16)

        if image_type == 'xy':
            width = LIDAR_CALIBRATION_WITHOUT_MANIPULATOR['fl_x'] - LIDAR_CALIBRATION_WITHOUT_MANIPULATOR['rr_x']
            height = LIDAR_CALIBRATION_WITHOUT_MANIPULATOR['fl_y'] - LIDAR_CALIBRATION_WITHOUT_MANIPULATOR['rr_y']
            rect = patches.Rectangle(
                (
                    LIDAR_CALIBRATION_WITHOUT_MANIPULATOR['rr_x'] * 1000,
                    LIDAR_CALIBRATION_WITHOUT_MANIPULATOR['rr_y'] * 1000
                ),
                width * 1000, height * 1000,
                linewidth=2, edgecolor='black', facecolor='none'
            )
            ax.add_patch(rect)

        if image_type == 'xy':
            first_points = x_points
            second_points = y_points
        elif image_type == 'xz':
            first_points = x_points
            second_points = z_points
        else:
            raise ValueError(f'unknown axis combination {image_type}')

        prev_first = 0.
        prev_second = 0.
        for i, first_points_i in enumerate(first_points):
            ax.plot(
                [prev_first, first_points_i],
                [prev_second, second_points[i]],
                c=color[i], linewidth=3
            )
            prev_first = first_points_i
            prev_second = second_points[i]

        ax.scatter(first_points, second_points, marker='o', s=15, c='b')

        ax.set_xlim(-crop_half_size, crop_half_size)
        ax.set_ylim(-crop_half_size, crop_half_size)
        ax.grid(which='both', linestyle='--', alpha=0.5)

        canvas.draw()
        jpeg = BytesIO()
        canvas.print_jpg(jpeg)
        result[image_type] = jpeg.getvalue()

    return result


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


if ENABLE_MULTIPROCESSING:
    import multiprocessing as mp

    if not mp.get_start_method(allow_none=True):
        mp.set_start_method('spawn')


class DrawInAnotherProcess:
    @classmethod
    def get_instance(cls, instance_name):
        instances = getattr(cls, 'instances', None)
        if instances is None:
            instances = {}
            setattr(cls, 'instances', instances)
        if instance_name in instances:
            instance = instances[instance_name]
        else:
            instance = cls(instance_name)
            instances[instance_name] = instance
        return instance

    @classmethod
    def stop_all(cls):
        instances = getattr(cls, 'instances', None)
        if instances is None:
            return
        for instance in instances.values():
            instance.stop()
        instances.clear()

    def __init__(self, name):
        self.conn, child_conn = mp.Pipe()
        self.process = mp.Process(
            name=f'DrawInAnotherProcess:{name}',
            target=self._processing_cycle,
            args=(child_conn,)
        )
        self.process.start()

    def draw(self, function_name, *params):
        self.conn.send([function_name, params])
        return self.conn.recv()

    def stop(self):
        self.conn.send(['stop', []])
        self.process.join()

    @staticmethod
    def _processing_cycle(conn):
        while True:
            function_name, params = conn.recv()
            if function_name == 'stop':
                conn.close()
                break
            result = globals()[function_name](*params)
            conn.send(result)
