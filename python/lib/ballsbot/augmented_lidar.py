from ballsbot.lidar import Lidar
from ballsbot.distance_sensors import DistanceSensors, has_distance_sensors


class AugmentedLidar:
    def __init__(self, lidar, distance_sensors):
        self.lidar = lidar
        self.distance_sensors = distance_sensors
        self.cached_distances = None
        self.sync_eps = 0.5  # seconds
        self.distances_merged_count = 0
        self.distances_ignored_count = 0
        self.distances_ignored_old_count = 0
        self.distances_ignored_new_count = 0

    def get_calibration(self):
        return self.lidar.get_calibration()

    def start(self):
        if self.distance_sensors is not None:
            self.distance_sensors.start()

    def get_lidar_points(self, range_limit=None, cached=False):
        nearby_points = self.lidar.get_lidar_points(range_limit=range_limit, cached=cached)
        if self.distance_sensors is not None:
            if not cached:
                self.cached_distances = self.distance_sensors.get_distances()
            if self.cached_distances:
                lidar_ts = self.get_points_ts()
                for it in self.cached_distances.values():
                    if not it or it['direction'] not in {"forward", "backward"}:
                        continue
                    if abs(lidar_ts - it['ts']) <= self.sync_eps:
                        x = it['distance'] if it['direction'] == "forward" else -it['distance']
                        y = it['offset_y']
                        nearby_points.append([x, y])
                        self.distances_merged_count += 1
                    else:
                        self.distances_ignored_count += 1
                        if lidar_ts > it['ts']:
                            self.distances_ignored_old_count += 1
                        else:
                            self.distances_ignored_new_count += 1

        return nearby_points

    def get_points_ts(self):
        return self.lidar.points_ts

    def get_track_frame(self):
        return {
            'points_ts': self.get_points_ts(),
            'distances': self.cached_distances,
            'augmented_points': self.get_lidar_points(cached=True),
            'distances_merged_count': self.distances_merged_count,
            'distances_ignored_count': self.distances_ignored_count,
            'distances_ignored_old_count': self.distances_ignored_old_count,
            'distances_ignored_new_count': self.distances_ignored_new_count,
        }


def get_augmented_lidar():
    lidar = Lidar()

    if has_distance_sensors():
        distance_sensors = DistanceSensors()
    else:
        distance_sensors = None

    return AugmentedLidar(lidar, distance_sensors)
