from matplotlib import patches
from matplotlib.transforms import Affine2D

from ballsbot.config import LIDAR_CALIBRATION_WITHOUT_MANIPULATOR


class BotPoseAbsCoordsDrawing:
    def __init__(self, dashboard, plot_name):
        self.plot_name = plot_name
        self.dashboard = dashboard

    @staticmethod
    def get_drawing_func_name():
        return 'draw_image_bot_pose_abs_coords'

    def update_image(self, poses, lidar_points, self_position, free_cells=None, target_point=None,
                     # pylint: disable=R0913
                     image_text=None, only_nearby_meters=6., additional_points=None):
        params = [
            poses, lidar_points, self_position, only_nearby_meters, free_cells, target_point, image_text,
            additional_points
        ]
        self.dashboard.set_subplot_data(self.plot_name, params)


def draw_image_bot_pose_abs_coords(
        ax, poses, lidar_points, self_position, only_nearby_meters,
        free_cells=None, target_point=None, image_text=None, additional_points=None,
):  # pylint: disable=R0913, R0914, C0103
    poses_x_points = [x['x'] for x in poses]
    poses_y_points = [x['y'] for x in poses]
    if len(poses) == 0:
        pose = {'x': 0., 'y': 0., 'teta': 0.}
    else:
        pose = poses[-1]

    if additional_points is not None:
        additional_x_points = [x[0] for x in additional_points]
        additional_y_points = [x[1] for x in additional_points]
        ax.scatter(additional_x_points, additional_y_points, marker='o', s=5, c='lightblue')

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

        rotation_center = ax.transData.transform([  # FIXME wrong position when teta = -pi
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


class BotPoseSelfCoordsDrawing:
    def __init__(self, dashboard, plot_name):
        self.plot_name = plot_name
        self.dashboard = dashboard

    @staticmethod
    def get_drawing_func_name():
        return 'draw_image_bot_pose_self_coords'

    def update_image(self, lidar_points, self_position, only_nearby_meters, additional_points):
        params = [lidar_points, self_position, only_nearby_meters, additional_points]
        self.dashboard.set_subplot_data(self.plot_name, params)


def draw_image_bot_pose_self_coords(
        ax, lidar_points, self_position, only_nearby_meters, additional_points
):  # pylint: disable=C0103
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


class ManipulatorPoseDrawing:
    def __init__(self, dashboard, plot_name_xy, plot_name_xz):
        self.plot_name_xy = plot_name_xy
        self.plot_name_xz = plot_name_xz
        self.dashboard = dashboard

    @staticmethod
    def get_drawing_func_name():
        return 'draw_image_manipulator_pose'

    def update_image(self, manipulator_pose, override_crop_half_size=None):
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

        params = ['xy', x_points, y_points, None, override_crop_half_size]
        self.dashboard.set_subplot_data(self.plot_name_xy, params)

        params = ['xz', x_points, None, z_points, override_crop_half_size]
        self.dashboard.set_subplot_data(self.plot_name_xz, params)


def draw_image_manipulator_pose(
        ax, image_type, x_points, y_points, z_points, override_crop_half_size=None
):  # pylint: disable=E1130, R0913, R0914, R0915, C0103
    if override_crop_half_size:
        crop_half_size = override_crop_half_size
    else:
        crop_half_size = 1.05

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


class StatusMessagesDrawing:
    def __init__(self, dashboard, plot_name):
        self.plot_name = plot_name
        self.dashboard = dashboard

    @staticmethod
    def get_drawing_func_name():
        return 'draw_image_status_messages'

    def update_image(self, text):
        params = [text]
        self.dashboard.set_subplot_data(self.plot_name, params)


def draw_image_status_messages(ax, text):  # pylint: disable=C0103
    ax.text(-1, 1, text, fontsize=16)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.axis('off')
