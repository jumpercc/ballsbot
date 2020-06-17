import time

# 9865, over rides only if needed, ie. TX2..
PCA9685_I2C_ADDR = 0x40
PCA9685_I2C_BUSNUM = 1

# STEERING
STEERING_CHANNEL = 0
THROTTLE_CHANNEL = 1


def map_range(x, X_min, X_max, Y_min, Y_max):
    '''
    Linear mapping between two ranges of values
    '''
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range / Y_range

    y = ((x - X_min) / XY_ratio + Y_min) // 1

    return int(y)


class PCA9685:
    '''
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    '''

    def __init__(self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
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
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pulse(pulse)


class PWMSteering:
    """
    Wrapper over a PWM motor controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self,
                 controller=None,
                 left_pulse=470,
                 right_pulse=320):
        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.pulse = map_range(0, self.LEFT_ANGLE, self.RIGHT_ANGLE,
                               self.left_pulse, self.right_pulse)
        self.running = True
        print('PWM Steering created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, angle):
        # map absolute angle to angle that vehicle can implement.
        self.pulse = map_range(angle,
                               self.LEFT_ANGLE, self.RIGHT_ANGLE,
                               self.left_pulse, self.right_pulse)

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

    def __init__(self,
                 controller=None,
                 max_pulse=390,
                 min_pulse=330,
                 zero_pulse=360):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.pulse = zero_pulse

        # send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)
        self.running = True

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, throttle):
        if abs(throttle) < 0.5:
            self.pulse = self.zero_pulse
        elif throttle > 0:
            if throttle > 0.9:
                self.pulse = self.max_pulse + 5
            else:
                self.pulse = self.max_pulse
        else:
            if throttle < -0.9:
                self.pulse = self.min_pulse - 5
            else:
                self.pulse = self.min_pulse

    def run(self, throttle):
        self.run_threaded(throttle)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # stop vehicle
        self.run(0)
        self.running = False


def get_controls():
    steering_controller = PCA9685(
        STEERING_CHANNEL,
        PCA9685_I2C_ADDR,
        busnum=PCA9685_I2C_BUSNUM
    )
    steering = PWMSteering(
        controller=steering_controller,
    )

    throttle_controller = PCA9685(
        THROTTLE_CHANNEL,
        PCA9685_I2C_ADDR, busnum=PCA9685_I2C_BUSNUM
    )
    throttle = PWMThrottle(
        controller=throttle_controller,
    )

    throttle.run(0)
    steering.run(0)

    return {
        'steering': steering,
        'throttle': throttle,
    }
