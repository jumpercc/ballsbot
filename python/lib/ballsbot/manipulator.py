from warnings import warn

from ballsbot.config import MANIPULATOR, SECONDS_TO_UNFOLD_MANIPULATOR
from ballsbot.servos import PCA9685, map_range
from ballsbot.utils import keep_rps, run_as_thread
from ballsbot_manipulator_geometry import ballsbot_manipulator_geometry

STEP = 0.003
IGNORE_LIMIT = 0.25


def get_controller_axis_value_getter(controller, axis, invert=False):
    def get_value():
        if len(controller.axes):
            value = controller.axes[axis].value
            if invert:
                value = -value

            if abs(value) > IGNORE_LIMIT:
                return float_map_range(value, -1, 1, -STEP, STEP)
        return 0.

    return get_value


def get_controller_two_buttons_value_getter(controller, button_up, button_down):
    def get_value():
        if len(controller.buttons):
            if controller.buttons[button_up].value:
                return STEP
            elif controller.buttons[button_down].value:
                return -STEP
        return 0.

    return get_value


def get_controller_stub():
    def do_nothing():
        return 0.

    return do_nothing()


def get_controller_link(controller, config):
    if not config:
        return get_controller_stub()
    elif "buttons" in config:
        return get_controller_two_buttons_value_getter(controller, *config['buttons'])
    elif "axis" in config:
        return get_controller_axis_value_getter(controller, config['axis'], invert=config.get('invert', False))
    raise NotImplementedError(config)


def float_map_range(x, x_min, x_max, y_min, y_max):
    x_range = x_max - x_min
    y_range = y_max - y_min
    xy_ratio = x_range / y_range
    return (x - x_min) / xy_ratio + y_min


class Manipulator:
    RPS = 50
    FOLD_STEPS = RPS * float(SECONDS_TO_UNFOLD_MANIPULATOR)
    _need_fold = False
    _need_unfold = False

    def __init__(self, controller, emulate_only=False):
        if MANIPULATOR is None or not MANIPULATOR['enabled']:
            warn("manipulator disabled by config")
            return

        self.emulate_only = emulate_only
        self.servo_values = {}
        self.servo_channel_to_index = {}
        for i, it in enumerate(MANIPULATOR['servos']):
            self.servo_values[it['channel']] = it['default_position']
            self.servo_channel_to_index[it['channel']] = i

        for servo_config in MANIPULATOR['servos']:
            run_as_thread(
                self._link_servo_to_function,
                servo=(None if emulate_only else PCA9685(servo_config['channel'])),
                servo_config=servo_config,
                get_value_cb=get_controller_link(controller, servo_config.get('control')),
            )

        run_as_thread(
            self._watch_fold,
            controller=controller,
            button_fold=MANIPULATOR['fold']['button_fold'],
            button_unfold=MANIPULATOR['fold']['button_unfold'],
            button_stop=MANIPULATOR['fold']['button_stop'],
        )

    @staticmethod
    def _watch_fold(controller, button_fold, button_unfold, button_stop):
        ts = None
        while True:
            ts = keep_rps(ts, fps=Manipulator.RPS)

            if len(controller.buttons):
                if controller.buttons[button_fold].value:
                    Manipulator._need_fold = True
                    Manipulator._need_unfold = False
                elif controller.buttons[button_unfold].value:
                    Manipulator._need_unfold = True
                    Manipulator._need_fold = False
                elif controller.buttons[button_stop].value:
                    Manipulator._need_fold = False
                    Manipulator._need_unfold = False

    @staticmethod
    def _get_fold_increment(need_position, current_position):
        return (need_position - current_position) / Manipulator.FOLD_STEPS

    def _link_servo_to_function(self, servo, servo_config, get_value_cb):
        current_position = servo_config['default_position']
        fold_increment = None

        ts = None
        while True:
            ts = keep_rps(ts, fps=Manipulator.RPS)

            if Manipulator._need_fold:
                if fold_increment is None:
                    fold_increment = self._get_fold_increment(servo_config['default_position'], current_position)
                if abs(servo_config['default_position'] - current_position) < fold_increment:
                    increment = servo_config['default_position'] - current_position
                    if increment == 0.:
                        Manipulator._need_fold = False
                        fold_increment = None
                else:
                    increment = fold_increment
            elif Manipulator._need_unfold:
                need_position = servo_config.get('unfold_position', servo_config['default_position'])
                if fold_increment is None:
                    fold_increment = self._get_fold_increment(need_position, current_position)
                if abs(need_position - current_position) < fold_increment:
                    increment = need_position - current_position
                    if increment == 0.:
                        Manipulator._need_unfold = False
                        fold_increment = None
                else:
                    increment = fold_increment
            else:
                fold_increment = None
                increment = get_value_cb()

            current_position += increment
            if current_position < -1.:
                current_position = -1.
                Manipulator._need_fold = False
                Manipulator._need_unfold = False
                fold_increment = None
            elif current_position > 1.:
                current_position = 1.
                Manipulator._need_fold = False
                Manipulator._need_unfold = False
                fold_increment = None

            self.servo_values[servo_config['channel']] = current_position
            pulse = map_range(
                current_position,
                -1., 1.,
                servo_config['min_pulse'], servo_config['max_pulse']
            )
            # TODO check feasible
            if not self.emulate_only:
                servo.set_pulse(pulse)

    def get_servo_positions(self):
        return {self.servo_channel_to_index[k]: v for k, v in self.servo_values.copy().items()}

    def get_manipulator_pose(self):
        servo_positions = self.get_servo_positions()
        default_points = []
        current_rotations = []
        for i, bone in enumerate(MANIPULATOR['bones']):
            if 'start' in bone:
                default_points.append(bone['default_position'])
                continue
            start_point = default_points[-1]
            servo = MANIPULATOR['servos'][i - 1]
            servo_position = servo_positions[i - 1]
            default_vector = bone['default_position']
            current_rotation = get_rotation(servo_position, servo, bone)
            end_point = add_vector_to_point(default_vector, start_point)
            default_points.append(end_point)
            current_rotations.append(current_rotation)

        points = ballsbot_manipulator_geometry.apply_rotations_wrapper(default_points, current_rotations)

        # TODO add magnetic encoders

        return {
            'points': points[:-2],
            'rotations': current_rotations,
            'claw_points': points[-2:],
        }


def get_rotation(servo_position, servo, bone):
    servo_default_position = servo['default_position']
    if servo_position == servo_default_position:
        return 0., 0., 0.
    rad_per_number = abs(servo['max_angle'] - servo['min_angle']) / 2.
    default_angle = servo_default_position * rad_per_number * bone['direction']
    current_angle = servo_position * rad_per_number * bone['direction']
    d_angle = current_angle - default_angle
    if bone['rotation'] == 'x':
        return d_angle, 0., 0.
    elif bone['rotation'] == 'y':
        return 0., -d_angle, 0.
    else:
        return 0., 0., d_angle


def add_vector_to_point(vector, point):
    x, y, z = point
    dx, dy, dz = vector
    return x + dx, y + dy, z + dz
