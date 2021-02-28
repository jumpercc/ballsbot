from warnings import warn

from ballsbot.config import MANIPULATOR
from ballsbot.servos import PCA9685, map_range
from ballsbot.utils import keep_rps, run_as_thread

STEP = 0.005
RPS = 50
IGNORE_LIMIT = 0.25


def link_servo_to_controller(servo, controller, axis, servo_config):
    current_position = servo_config['default_position']

    ts = None
    while True:
        ts = keep_rps(ts, fps=RPS)

        if len(controller.axes):
            value = controller.axes[axis].value

            if abs(value) > IGNORE_LIMIT:
                increment = float_map_range(value, -1, 1, -STEP, STEP)
                current_position += increment
                if current_position < -1.:
                    current_position = -1.
                elif current_position > 1.:
                    current_position = 1.

        pulse = map_range(
            current_position,
            -1., 1.,
            servo_config['min_pulse'], servo_config['max_pulse']
        )
        servo.set_pulse(pulse)


def float_map_range(x, X_min, X_max, Y_min, Y_max):
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range / Y_range
    return ((x - X_min) / XY_ratio + Y_min)


class Manipulator:
    def __init__(self, controller):
        if MANIPULATOR is None or not MANIPULATOR['enabled']:
            warn("manipulator disabled by config")
            return

        servo_config = MANIPULATOR['servos'][0]
        axis = 1
        run_as_thread(
            link_servo_to_controller,
            servo=PCA9685(servo_config['channel']),
            controller=controller,
            axis=axis,
            servo_config=servo_config
        )

        servo_config = MANIPULATOR['servos'][1]
        axis = 0
        run_as_thread(
            link_servo_to_controller,
            servo=PCA9685(servo_config['channel']),
            controller=controller,
            axis=axis,
            servo_config=servo_config
        )

        servo_config = MANIPULATOR['servos'][2]
        axis = 2
        run_as_thread(
            link_servo_to_controller,
            servo=PCA9685(servo_config['channel']),
            controller=controller,
            axis=axis,
            servo_config=servo_config
        )
