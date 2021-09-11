import time
from ballsbot.config import PCA9685_I2C_BUSNUM, PCA9685_I2C_ADDR, CAR_CONTROLS


def map_range(x, x_min, x_max, y_min, y_max):
    """
    Linear mapping between two ranges of values
    """
    x_range = x_max - x_min
    y_range = y_max - y_min
    xy_ratio = x_range / y_range

    y = ((x - x_min) / xy_ratio + y_min) // 1

    return int(y)


def scaled_map_range(value, from_min, from_max, to_min, to_max, scale=1.):  # pylint: disable=R0913
    result = map_range(value, from_min / scale, from_max / scale, to_min, to_max)

    if to_min > to_max:
        to_min, to_max = to_max, to_min

    if result < to_min:
        result = to_min
    elif result > to_max:
        result = to_max

    return result


class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    # pylint: disable=R0913
    def __init__(self, channel, address=PCA9685_I2C_ADDR, frequency=60, busnum=PCA9685_I2C_BUSNUM, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685  # pylint: disable=C0415
        # Initialise the PCA9685 using the default address (PCA9685_I2C_ADDR).
        if busnum is not None:
            from Adafruit_GPIO import I2C  # pylint: disable=C0415

            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise

    def set_pulse(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:  # pylint: disable=W0702
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pulse(pulse)


class PWMSteering:
    """
    Wrapper over a PWM motor controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, *, controller, config):
        self.controller = controller
        self.left_pulse = config['min_pulse']
        self.right_pulse = config['max_pulse']
        self.scale = 2.
        self.pulse = scaled_map_range(
            0,
            self.LEFT_ANGLE, self.RIGHT_ANGLE,
            self.left_pulse, self.right_pulse,
            self.scale
        )
        self.running = True
        print('PWM Steering created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, angle):
        # map absolute angle to angle that vehicle can implement.
        self.pulse = scaled_map_range(
            angle,
            self.LEFT_ANGLE, self.RIGHT_ANGLE,
            self.left_pulse, self.right_pulse,
            self.scale
        )

    def run(self, angle):
        self.run_threaded(angle)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # set steering straight
        self.pulse = 0
        time.sleep(0.3)
        self.running = False


class PWMThrottle:
    """
    Wrapper over a PWM motor controller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self, *, controller, config):
        self.controller = controller
        self.max_turbo_pulse = config['max_turbo_pulse']
        self.max_pulse = config['max_pulse']
        self.zero_pulse = config['zero_pulse']
        self.min_pulse = config['min_pulse']
        self.min_turbo_pulse = config['min_turbo_pulse']
        self.pulse = self.zero_pulse
        self.scale = 2.
        self.throttle = 0.
        self.zero_throttle = map_range(
            self.zero_pulse, self.min_pulse, self.max_pulse, self.MIN_THROTTLE, self.MAX_THROTTLE)

        # send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)
        self.running = True

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, throttle, turbo):
        # self.throttle = throttle
        # self.pulse = map_range(throttle,
        #                        self.MIN_THROTTLE, self.MAX_THROTTLE,
        #                        self.min_pulse, self.max_pulse)
        if abs(throttle - self.zero_throttle) < 0.25:
            self.throttle = 0.
            self.pulse = self.zero_pulse
        elif turbo:
            self.pulse = scaled_map_range(
                throttle,
                self.MIN_THROTTLE, self.MAX_THROTTLE,
                self.min_turbo_pulse, self.max_turbo_pulse,
                self.scale
            )
        else:
            self.pulse = scaled_map_range(
                throttle,
                self.MIN_THROTTLE, self.MAX_THROTTLE,
                self.min_pulse, self.max_pulse,
                self.scale
            )

    def run(self, throttle, turbo=False):
        self.run_threaded(throttle, turbo)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # stop vehicle
        self.run(0)
        self.running = False


_car_controls = None


def get_controls():
    global _car_controls
    if not _car_controls:
        steering = PWMSteering(
            controller=PCA9685(CAR_CONTROLS['steering']['channel']),
            config=CAR_CONTROLS['steering'],
        )

        throttle = PWMThrottle(
            controller=PCA9685(CAR_CONTROLS['throttle']['channel']),
            config=CAR_CONTROLS['throttle'],
        )

        throttle.run(0)
        steering.run(0)

        _car_controls = {
            'steering': steering,
            'throttle': throttle,
        }
    return _car_controls
