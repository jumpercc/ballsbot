from warnings import warn

from ballsbot.config import MANIPULATOR
from ballsbot.servos import PCA9685, map_range
from ballsbot.utils import keep_rps, run_as_thread

STEP = 0.005
IGNORE_LIMIT = 0.25


def get_controller_axis_value_getter(controller, axis):
    def get_value():
        if len(controller.axes):
            value = controller.axes[axis].value

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

    def __init__(self, controller):
        if MANIPULATOR is None or not MANIPULATOR['enabled']:
            warn("manipulator disabled by config")
            return

        servo_config = MANIPULATOR['servos'][0]
        run_as_thread(
            self._link_servo_to_function,
            servo=PCA9685(servo_config['channel']),
            servo_config=servo_config,
            get_value_cb=get_controller_two_buttons_value_getter(controller, 5, 7),
        )

        servo_config = MANIPULATOR['servos'][1]
        axis = 3
        run_as_thread(
            self._link_servo_to_function,
            servo=PCA9685(servo_config['channel']),
            servo_config=servo_config,
            get_value_cb=get_controller_axis_value_getter(controller, axis),
        )

        servo_config = MANIPULATOR['servos'][2]
        axis = 2
        run_as_thread(
            self._link_servo_to_function,
            servo=PCA9685(servo_config['channel']),
            servo_config=servo_config,
            get_value_cb=get_controller_axis_value_getter(controller, axis),
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
    def _link_servo_to_function(servo, servo_config, get_value_cb):
        current_position = servo_config['default_position']
        fold_increment = None

        ts = None
        while True:
            ts = keep_rps(ts, fps=Manipulator.RPS)

            if Manipulator._need_fold:
                if fold_increment is None:
                    fold_increment = (servo_config['default_position'] - current_position) / Manipulator.FOLD_STEPS
                if abs(servo_config['default_position'] - current_position) < fold_increment:
                    increment = 0.
                    Manipulator._need_fold = False
                    fold_increment = None
                else:
                    increment = fold_increment
            elif Manipulator._need_unfold:
                need_position = servo_config.get('unfold_position', servo_config['default_position'])
                if fold_increment is None:
                    fold_increment = (need_position - current_position) / Manipulator.FOLD_STEPS
                if abs(need_position - current_position) < fold_increment:
                    increment = 0.
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

            pulse = map_range(
                current_position,
                -1., 1.,
                servo_config['min_pulse'], servo_config['max_pulse']
            )
            servo.set_pulse(pulse)
