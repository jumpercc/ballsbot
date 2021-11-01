from ballsbot.config import MANIPULATOR
from ballsbot.servos import PCA9685, map_range
from ballsbot.utils import keep_rps, run_as_thread
from ballsbot_manipulator_geometry import ballsbot_manipulator_geometry

RPS = 50
STEP = 0.0016
IGNORE_DIFF = 0.01


def float_map_range(x, x_min, x_max, y_min, y_max):
    x_range = x_max - x_min
    y_range = y_max - y_min
    xy_ratio = x_range / y_range
    return (x - x_min) / xy_ratio + y_min


class Manipulator:
    def __init__(self, joystick_wrapper, emulate_only=False):
        if MANIPULATOR is None or not MANIPULATOR['enabled']:
            return

        self.joystick_wrapper = joystick_wrapper
        self.emulate_only = emulate_only
        self.servo_values = {}
        self.servo_channel_to_index = {}
        self.servos = []
        for servo_number, servo_config in enumerate(MANIPULATOR['servos']):
            self.servo_values[servo_config['channel']] = servo_config['default_position']
            self.servo_channel_to_index[servo_config['channel']] = servo_number
            self.servos.append(None if emulate_only else PCA9685(servo_config['channel']))

        run_as_thread(self._start)

    def _start(self):
        ts = None
        while True:
            ts = keep_rps(ts, fps=RPS)

            if self.joystick_wrapper.manipulator_fold_pressed:
                directions = [0 for _ in MANIPULATOR['servos']]
                for servo_number, servo_config in enumerate(MANIPULATOR['servos']):
                    current_position = self.servo_values[servo_config['channel']]
                    diff = current_position - servo_config['default_position']
                    if abs(diff) <= IGNORE_DIFF:
                        directions[servo_number] = 0
                    elif diff > 0:
                        directions[servo_number] = -1
                    else:
                        directions[servo_number] = 1
            elif self.joystick_wrapper.manipulator_unfold_pressed:
                directions = [0 for _ in MANIPULATOR['servos']]
                for servo_number, servo_config in enumerate(MANIPULATOR['servos']):
                    current_position = self.servo_values[servo_config['channel']]
                    diff = current_position - servo_config.get('unfold_position', servo_config['default_position'])
                    if abs(diff) <= IGNORE_DIFF:
                        directions[servo_number] = 0
                    elif diff > 0:
                        directions[servo_number] = -1
                    else:
                        directions[servo_number] = 1
            else:
                directions = self.joystick_wrapper.manipulator_directions.copy()

            set_pulse_list = []
            for servo_number, direction in enumerate(directions):
                if direction:
                    self._apply_direction(servo_number, direction)
                    set_pulse_list.append(True)
                else:
                    set_pulse_list.append(False)

            if not self.emulate_only:
                for servo_number, set_pulse in enumerate(set_pulse_list):
                    if not set_pulse:
                        continue
                    servo_config = MANIPULATOR['servos'][servo_number]
                    current_position = self.servo_values[servo_config['channel']]
                    pulse = map_range(
                        current_position,
                        -1., 1.,
                        servo_config['min_pulse'], servo_config['max_pulse']
                    )
                    servo = self.servos[servo_number]
                    servo.set_pulse(pulse)

    def _apply_direction(self, servo_number, direction):
        servo_config = MANIPULATOR['servos'][servo_number]
        current_position = self.servo_values[servo_config['channel']]

        current_position += STEP if direction > 0 else -STEP
        if current_position < -1.:
            current_position = -1.
        elif current_position > 1.:
            current_position = 1.
        # TODO check feasible

        self.servo_values[servo_config['channel']] = current_position

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
