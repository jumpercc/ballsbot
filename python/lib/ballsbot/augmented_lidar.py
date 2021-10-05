from math import pi

from ballsbot.utils import keep_rps
from ballsbot.lidar import Lidar, radial_points_to_cartesian
from ballsbot.distance_sensors import DistanceSensors, has_distance_sensors


class AugmentedLidar:
    def __init__(self, lidar, distance_sensors):
        self.lidar = lidar
        self.distance_sensors = distance_sensors
        self.cached_distances = None

    def start(self):
        if self.distance_sensors is not None:
            self.distance_sensors.start()

    def get_radial_lidar_points(self, range_limit=None, cached=True):
        nearby_points = self.lidar.get_radial_lidar_points(range_limit, cached=cached)

        if self.distance_sensors is not None:
            self.cached_distances = self.distance_sensors.get_distances()
            for sensor_name, direction_name in self.distance_sensors.get_directions().items():
                angle = 0. if direction_name == "forward" else pi  # TODO other directions
                a_distance = self.cached_distances.get(sensor_name)
                if a_distance is not None:
                    a_distance /= 1000.  # to meters
                    nearby_points.append({'distance': a_distance, 'angle': angle})

        return nearby_points

    def _get_lidar_points(self):
        points = self.get_radial_lidar_points(cached=False)
        points = radial_points_to_cartesian(points)
        return points

    def get_lidar_points(self):
        return self.lidar.get_lidar_points()

    def show_lidar_cloud(self, image, **kwargs):
        self.auto_update_lidar_cloud(
            cb=lambda points, only_nearby_meters: self.lidar._update_picture(  # pylint: disable=W0212
                image, points, only_nearby_meters=only_nearby_meters
            ),
            **kwargs
        )

    def auto_update_lidar_cloud(self, only_nearby_meters=4, fps=2, cb=None):
        ts = None
        while True:
            ts = keep_rps(ts, fps=fps)
            points = self._get_lidar_points()
            if len(points) == 0:
                break

            if cb is not None:
                cb(points, only_nearby_meters=only_nearby_meters)


def get_augmented_lidar():
    lidar = Lidar()

    if has_distance_sensors():
        distance_sensors = DistanceSensors(autostart=True)
    else:
        distance_sensors = None

    return AugmentedLidar(lidar, distance_sensors)
