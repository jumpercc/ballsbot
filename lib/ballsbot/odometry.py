from math import sin, cos, pi


class Odometry:
    def __init__(self, imu, throttle, speed=0.45):
        self.imu = imu
        self.throttle = throttle
        self.prev = None
        self.teta_eps = 0.01
        self.teta_speed = 0.08
        self.speed = speed
        self.readings = []

    def get_dx_dy(self, dt, teta, keep_readings=False):
        current = {
            'teta': teta,
            'throttle': self.throttle.throttle,
        }

        if keep_readings:
            self.readings.append(current)

        if self.prev is None:
            result = [0., 0.]
        else:
            dteta = teta - self.prev['teta']
            if dteta > 2 * pi:
                dteta -= 2 * pi
            elif dteta < -2 * pi:
                dteta += 2 * pi
            current['dteta'] = dteta

            if self.prev['throttle'] == 0:
                direction = 0.
            elif self.prev['throttle'] > 0.:
                direction = 1.
            else:
                direction = -1.

            if direction == 0.:
                dx = 0.
                dy = 0.
            else:
                speed = self.speed
                if abs(dteta) > self.teta_speed:
                    speed *= 0.5
                # TODO w from imu
                dx = direction * speed * cos(teta + dteta) * dt
                dy = direction * speed * sin(teta + dteta) * dt

            result = [dx, dy]

        self.prev = current
        return result
