from ballsbot.config import MANIPULATOR, CAR_CONTROLS
from ballsbot.utils import keep_rps, run_as_thread

IGNORE_LIMIT = 0.25


class JoystickWrapperBase:
    def __init__(self):
        self.steering = 0.
        self.throttle = 0.
        self.turbo_pressed = 0

        self.manipulator_fold_pressed = 0
        self.manipulator_unfold_pressed = 0
        self.manipulator_directions = [0 for _ in MANIPULATOR.get('servos', [])]

    def get_state(self):
        return {
            'steering': self.steering,
            'throttle': self.throttle,
            'turbo_pressed': self.turbo_pressed,
            'manipulator_fold_pressed': self.manipulator_fold_pressed,
            'manipulator_unfold_pressed': self.manipulator_unfold_pressed,
            'manipulator_directions': self.manipulator_directions.copy(),
        }

    def stop(self):
        self.steering = 0.
        self.throttle = 0.
        self.turbo_pressed = 0

        self.manipulator_fold_pressed = 0
        self.manipulator_unfold_pressed = 0
        self.manipulator_directions = [0 for _ in MANIPULATOR.get('servos', [])]


class JoystickMock:
    class JoystickMockButton:
        def __init__(self):
            self.value = None

        def set_value(self, value):
            self.value = value

    def __init__(self, axes_count=4, buttons_count=17):
        self.axes = [self.JoystickMockButton() for _ in range(axes_count)]
        self.buttons = [self.JoystickMockButton() for _ in range(buttons_count)]

    def set_values(self, axes_values, buttons_values):
        assert len(axes_values) == len(self.axes), f'axes: {len(axes_values)} != {len(self.axes)}'
        assert len(buttons_values) == len(self.buttons), f'buttons: {len(buttons_values)} != {len(self.buttons)}'
        for i, value in enumerate(axes_values):
            self.axes[i].set_value(value)
        for i, value in enumerate(buttons_values):
            self.buttons[i].set_value(value)

    def stop(self):
        for it in self.buttons:
            it.set_value(0.)
        for it in self.axes:
            it.set_value(0.)


class JoystickWrapperBaseWithUpdates(JoystickWrapperBase):
    def __init__(self, joystick):
        super().__init__()
        self.joystick = joystick

    def update_all(self):
        self.update_car_controls()
        self._update_manipulator_controls()

    def update_car_controls(self):
        steering_config = CAR_CONTROLS['steering']['control']
        throttle_config = CAR_CONTROLS['throttle']['control']
        invert_throttle = throttle_config.get('invert')
        turbo_button = CAR_CONTROLS['throttle']['turbo_control']['button']

        self.steering = self.joystick.axes[steering_config['axis']].value

        value = self.joystick.axes[throttle_config['axis']].value
        self.throttle = -value if invert_throttle else value

        self.turbo_pressed = self.joystick.buttons[turbo_button].value

    def _update_manipulator_controls(self):
        if MANIPULATOR is None or not MANIPULATOR['enabled']:
            return

        for servo_number, servo_config in enumerate(MANIPULATOR['servos']):
            self._update_one_servo_controls(servo_number, servo_config.get('control'))

        self.manipulator_fold_pressed = self.joystick.buttons[MANIPULATOR['fold']['button_fold']].value
        self.manipulator_unfold_pressed = self.joystick.buttons[MANIPULATOR['fold']['button_unfold']].value

    def _update_one_servo_controls(self, servo_number, servo_config):
        if "buttons" in servo_config:
            return self._update_two_buttons_value_getter(servo_number, *servo_config['buttons'])
        elif "axis" in servo_config:
            return self._update_axis_value_getter(
                servo_number, servo_config['axis'], invert=servo_config.get('invert', False))
        raise NotImplementedError(servo_config)

    def _update_axis_value_getter(self, servo_number, axis, invert=False):
        value = self.joystick.axes[axis].value
        if invert:
            value = -value

        if abs(value) <= IGNORE_LIMIT:
            value = 0
        elif value > 0:
            value = 1
        else:
            value = -1

        self.manipulator_directions[servo_number] = value

    def _update_two_buttons_value_getter(self, servo_number, button_up, button_down):
        up_value = self.joystick.buttons[button_up].value
        down_value = self.joystick.buttons[button_down].value

        if up_value and not down_value:
            self.manipulator_directions[servo_number] = 1
        elif not up_value and down_value:
            self.manipulator_directions[servo_number] = -1
        else:
            self.manipulator_directions[servo_number] = 0


class JoystickWrapper(JoystickWrapperBaseWithUpdates):
    def __init__(self, joystick):
        super().__init__(joystick)
        self.running = True

        run_as_thread(self._start)

    def stop(self):
        self.running = False

    def _start(self):
        ts = None
        while self.running:
            ts = keep_rps(ts, fps=1)
            if len(self.joystick.axes):
                break

        ts = None
        while self.running:
            ts = keep_rps(ts, fps=10)
            self.update_all()
