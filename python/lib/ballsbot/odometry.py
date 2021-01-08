from math import sin, cos, pi
from time import time
from ballsbot.utils import run_as_thread


class Odometry:
    def __init__(self, imu, throttle):
        self.imu = imu
        self.throttle = throttle
        self.prev = None
        self.teta_eps = 0.01
        self.teta_speed = 0.08

        self.odometry_meters_per_rotation = 3.5 / 39.
        self.odometry_counter = 0
        self.direction = 0.
        self.odometry_last_interval = None
        run_as_thread(self.update_odometry_cycle)

    def get_dx_dy(self, dt, teta):
        current = {
            'teta': teta,
            'throttle': self.throttle.throttle,
            'w_z': self.imu.get_w_z(),
            'dt': dt,
        }

        if self.prev is None:
            result = [0., 0.]
        else:
            dteta = teta - self.prev['teta']
            if dteta > pi:
                dteta -= 2 * pi
            elif dteta < -pi:
                dteta += 2 * pi
            current['dteta'] = dteta

            if self.get_speed() == 0.:
                result = [0., 0.]
            else:
                if self.direction != 0.:
                    direction = self.direction
                else:
                    direction = 1. if self.prev['throttle'] > 0. else -1.

                speed = direction * self.get_speed()
                if abs(dteta) < self.teta_eps:
                    dx = speed * cos(teta) * dt
                    dy = speed * sin(teta) * dt
                else:
                    w_z = current['w_z']
                    prev_teta = self.prev['teta']
                    if w_z == 0.:
                        dx = dy = 0.
                    else:
                        dx = speed / w_z * (-sin(prev_teta) + sin(prev_teta + w_z * dt))
                        dy = speed / w_z * (cos(prev_teta) - cos(prev_teta + w_z * dt))
                result = [dx, dy]

        self.prev = current
        return result

    def update_odometry_cycle(self):
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi

        # input_pin_speed = 18  # BCM pin 18, BOARD pin 12
        input_pin_speed = 20  # BCM pin 20, BOARD pin 38
        GPIO.setup(input_pin_speed, GPIO.IN)

        input_pin_direction = 19  # BCM pin 19, BOARD pin 35
        GPIO.setup(input_pin_direction, GPIO.IN)

        try:
            prev_ts = time()
            while True:
                if GPIO.wait_for_edge(input_pin_speed, GPIO.FALLING, timeout=750) is None:
                    ts = time()
                    self.odometry_last_interval = None
                    self.direction = 0.
                else:
                    self.direction = 1. if GPIO.input(input_pin_direction) == GPIO.HIGH else -1.
                    self.odometry_counter += 1
                    ts = time()
                    if 0.01 < ts - prev_ts < 10.:
                        self.odometry_last_interval = ts - prev_ts
                prev_ts = ts
        finally:
            GPIO.cleanup()

    def get_speed(self):
        if self.odometry_last_interval is not None:
            return self.odometry_meters_per_rotation / self.odometry_last_interval
        return 0.

    def get_direction(self):
        return self.direction
