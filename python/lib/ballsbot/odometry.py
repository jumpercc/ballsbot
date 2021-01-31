from math import sin, cos, pi
from time import time
from ballsbot.utils import run_as_thread, keep_rps
from ballsbot.config import ODOMETRY_METERS_PER_ROTATION, ODOMETRY_PRIMARY_PIN, ODOMETRY_SECONDARY_PIN


class Odometry:
    def __init__(self, imu, throttle):
        self.imu = imu
        self.throttle = throttle
        self.prev = None
        self.teta_eps = 0.01
        self.teta_speed = 0.08

        self.odometry_meters_per_rotation = ODOMETRY_METERS_PER_ROTATION
        self.odometry_counter = 0
        self.direction = 0.
        self.odometry_last_interval = None

        self.event_timestamps = {
            'primary_rising': None,
            'primary_falling': None,
            'secondary_rising': None,
            'secondary_falling': None,
            'any_event': None,
        }

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

        GPIO.setup(ODOMETRY_PRIMARY_PIN, GPIO.IN)
        GPIO.setup(ODOMETRY_SECONDARY_PIN, GPIO.IN)

        detections = self.event_timestamps

        def primary_both(_):
            if GPIO.input(ODOMETRY_PRIMARY_PIN) == GPIO.HIGH:
                detections['primary_rising'] = time()
                detections['any_event'] = detections['primary_rising']
            else:
                ts = time()
                if detections['primary_falling'] is not None:
                    prev_ts = detections['primary_falling']
                    an_interval = ts - prev_ts
                    if 0.05 < an_interval < 2.:
                        self.odometry_last_interval = an_interval
                detections['primary_falling'] = ts
                detections['any_event'] = detections['primary_falling']

        def secondary_both(_):
            if GPIO.input(ODOMETRY_SECONDARY_PIN) == GPIO.HIGH:
                detections['secondary_rising'] = time()
                detections['any_event'] = detections['secondary_rising']
                self.odometry_counter += 1
            else:
                detections['secondary_falling'] = time()
                detections['any_event'] = detections['secondary_falling']

        try:
            GPIO.add_event_detect(ODOMETRY_PRIMARY_PIN, GPIO.BOTH, callback=primary_both)
            GPIO.add_event_detect(ODOMETRY_SECONDARY_PIN, GPIO.BOTH, callback=secondary_both)

            ts = None
            while True:
                ts = keep_rps(ts, fps=2)
                if detections['any_event'] is None or time() - detections['any_event'] > 0.5:
                    # reset state if no events (car stopped)
                    self.odometry_last_interval = None
                    self.direction = 0.
                    for k in detections.keys():
                        detections[k] = None
                elif detections['primary_falling'] is not None and detections['secondary_rising'] is not None \
                        and (detections['primary_rising'] is not None or detections['secondary_falling'] is not None):
                    # select events to determine direction
                    select_rising = False
                    if detections['primary_rising'] is not None and detections['secondary_falling'] is not None:
                        if abs(detections['primary_rising'] - detections['secondary_rising']) < \
                                abs(detections['primary_falling'] - detections['secondary_falling']):
                            select_rising = True
                        else:
                            select_rising = False
                    # determine direction
                    if select_rising and detections['primary_rising'] is not None:
                        self.direction = 1. if detections['primary_rising'] < detections['secondary_rising'] else -1.
                    else:
                        self.direction = 1. if detections['primary_falling'] < detections['secondary_falling'] else -1.
        finally:
            GPIO.remove_event_detect(ODOMETRY_PRIMARY_PIN)
            GPIO.remove_event_detect(ODOMETRY_SECONDARY_PIN)
            GPIO.cleanup()

    def get_speed(self):
        if self.odometry_last_interval is not None:
            return self.odometry_meters_per_rotation / self.odometry_last_interval
        return 0.

    def get_direction(self):
        return self.direction
