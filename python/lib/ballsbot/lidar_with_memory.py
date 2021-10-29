from ballsbot.augmented_lidar import AugmentedLidar, get_augmented_lidar
from ballsbot_localization import ballsbot_localization as grid


class LidarWithMemory:
    def __init__(self, lidar=None, distance_sensors=None):
        if lidar:
            self.lidar = AugmentedLidar(lidar, distance_sensors)
        else:
            self.lidar = get_augmented_lidar()
        self.update_grid_ts = None

    def get_calibration(self):
        return self.lidar.get_calibration()

    def start(self):
        self.lidar.start()

    def tick(self, pose):
        """
        use tick or tick_get_points+tick_update_grid
        """
        it = self.tick_get_points()
        self.tick_update_grid(pose, it['points'], it['ts'])

    def tick_get_points(self):
        """
        use tick or tick_get_points+tick_update_grid
        """
        return {
            'points': self.lidar.get_lidar_points(cached=False),
            'ts': self.lidar.get_points_ts(),
        }

    def tick_update_grid(self, pose, points, points_ts):
        """
        use tick or tick_get_points+tick_update_grid
        """
        if points is None:
            grid.update_pose(pose)
        else:
            grid.update_grid(points, pose, points_ts)  # TODO update pose from fixed
            grid.clean_up_grid(points_ts)
        self.update_grid_ts = points_ts

    def get_lidar_points(self, range_limit=0., cached=True, absolute_coords=True):  # pylint: disable=W0613
        if self.update_grid_ts:
            return grid.get_sparse_point_cloud(self.update_grid_ts, range_limit, absolute_coords)
        else:
            return None

    def get_points_ts(self):
        return self.update_grid_ts

    def get_track_frame(self):
        result = self.lidar.get_track_frame()
        if self.update_grid_ts:
            pose = grid.get_poses()[-1]
        else:
            pose = None
        result.update({
            'sparse_points': self.get_lidar_points(absolute_coords=False),
            'update_grid_ts': self.update_grid_ts,
            'grid_pose': pose,
        })
        return result
