from math import sin, cos, pi
import RPi.GPIO as GPIO
from time import time
from ballsbot.utils import run_as_thread


class Odometry:
    def __init__(self, imu, throttle, speed=0.45):
        self.imu = imu
        self.throttle = throttle
        self.prev = None
        self.teta_eps = 0.01
        self.teta_speed = 0.08
        self.default_speed = speed
        self.readings = []

        self.odometry_meters_per_rotation = 3.5 / 39.
        self.odometry_counter = 0
        self.odometry_last_interval = None
        run_as_thread(self.update_odometry_cycle)

    def get_dx_dy(self, dt, teta, keep_readings=False):
        current = {
            'teta': teta,
            'throttle': self.throttle.throttle,
            'w_z': self.imu.get_w_z(),
            'dt': dt,
        }

        if keep_readings:
            self.readings.append(current)

        if self.prev is None:
            result = [0., 0.]
        else:
            dteta = teta - self.prev['teta']
            if dteta > pi:
                dteta -= 2 * pi
            elif dteta < -pi:
                dteta += 2 * pi
            current['dteta'] = dteta

            if self.prev['throttle'] == 0.:
                result = [0., 0.]
            else:
                if self.prev['throttle'] > 0.:
                    direction = 1.
                else:
                    direction = -1.

                speed = direction * self.get_speed()
                if abs(dteta) < self.teta_eps:
                    dx = speed * cos(teta) * dt
                    dy = speed * sin(teta) * dt
                    if keep_readings:
                        current['speed'] = speed
                        current['dx'] = dx
                        current['dy'] = dy
                else:
                    if abs(dteta) > self.teta_speed and speed == self.default_speed:
                        speed *= 0.5
                    w_z = current['w_z']
                    prev_teta = self.prev['teta']
                    dx = speed / w_z * (-sin(prev_teta) + sin(prev_teta + w_z * dt))
                    dy = speed / w_z * (cos(prev_teta) - cos(prev_teta + w_z * dt))
                    if keep_readings:
                        dteta_test = w_z * dt
                        current['dteta_test'] = dteta_test
                        current['w_z'] = w_z
                        current['speed'] = speed
                if keep_readings:
                    current['dx'] = dx
                    current['dy'] = dy
                result = [dx, dy]

        self.prev = current
        return result

    def update_odometry_cycle(self):
        input_pin = 18  # BCM pin 18, BOARD pin 12
        GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
        GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin

        try:
            prev_ts = time()
            while True:
                GPIO.wait_for_edge(input_pin, GPIO.FALLING)
                self.odometry_counter += 1
                ts = time()
                if 0.01 < ts - prev_ts < 10.:
                    self.odometry_last_interval = ts - prev_ts
                else:
                    pass  # TODO warning
                prev_ts = ts
        finally:
            GPIO.cleanup()

    def get_speed(self):
        if self.odometry_last_interval is not None:
            return self.odometry_meters_per_rotation / self.odometry_last_interval
        return self.default_speed
