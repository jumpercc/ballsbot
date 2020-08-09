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

                speed = self.speed
                if abs(dteta) < self.teta_eps:
                    dx = direction * speed * cos(teta) * dt
                    dy = direction * speed * sin(teta) * dt
                else:
                    if abs(dteta) > self.teta_speed:
                        speed *= 0.5
                    w_z = current['w_z']
                    prev_teta = self.prev['teta']
                    dx = abs(speed / w_z) * (-sin(prev_teta) + sin(prev_teta + w_z * dt))
                    dy = abs(speed / w_z) * (cos(prev_teta) - cos(prev_teta + w_z * dt))
                    if keep_readings:
                        dteta_test = w_z * dt
                        current['dteta_test'] = dteta_test
                        current['w_z'] = w_z

                result = [dx, dy]

        self.prev = current
        return result
