from warnings import warn

from ballsbot.config import MANIPULATOR
from ballsbot.servos import PCA9685, map_range
from ballsbot.utils import keep_rps, run_as_thread
from ballsbot_manipulator import ballsbot_manipulator

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


def float_map_range(x, X_min, X_max, Y_min, Y_max):
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range / Y_range
    return ((x - X_min) / XY_ratio + Y_min)


class Manipulator:
    RPS = 50
    FOLD_STEPS = RPS * 2.  # 2 seconds
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

        servo_config = MANIPULATOR['servos'][0]
        run_as_thread(
            self._link_servo_to_function,
            servo=(None if emulate_only else PCA9685(servo_config['channel'])),
            servo_config=servo_config,
            get_value_cb=get_controller_two_buttons_value_getter(controller, 5, 7),
        )

        servo_config = MANIPULATOR['servos'][1]
        axis = 3
        run_as_thread(
            self._link_servo_to_function,
            servo=(None if emulate_only else PCA9685(servo_config['channel'])),
            servo_config=servo_config,
            get_value_cb=get_controller_axis_value_getter(controller, axis, invert=True),
        )

        servo_config = MANIPULATOR['servos'][2]
        axis = 2
        run_as_thread(
            self._link_servo_to_function,
            servo=(None if emulate_only else PCA9685(servo_config['channel'])),
            servo_config=servo_config,
            get_value_cb=get_controller_axis_value_getter(controller, axis),
        )

        servo_config = MANIPULATOR['servos'][3]
        run_as_thread(
            self._link_servo_to_function,
            servo=(None if emulate_only else PCA9685(servo_config['channel'])),
            servo_config=servo_config,
            get_value_cb=get_controller_two_buttons_value_getter(controller, 4, 6),
        )

        run_as_thread(
            self._watch_fold,
            controller=controller,
            button_fold=0,
            button_unfold=3,
            button_stop=2,
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

            current_position += increment  # FIXME need to move right to end point
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

        points = ballsbot_manipulator.apply_rotations_wrapper(default_points, current_rotations)

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
