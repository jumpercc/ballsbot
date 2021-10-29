from ballsbot import drawing
from ballsbot.utils import keep_rps
from ballsbot.lidar import calibration_to_xywh


class LidarDrawing:
    def __init__(self, lidar):
        self.lidar = lidar
        self.position = calibration_to_xywh(lidar.get_calibration())

    def _update_picture(self, image, points, only_nearby_meters=4, additional_points=None):
        drawing.update_picture_self_coords(image, points, self.position, only_nearby_meters, additional_points)

    def _auto_update_lidar_cloud(self, only_nearby_meters=4, fps=2, cb=None, cached=True):
        ts = None
        while True:
            ts = keep_rps(ts, fps=fps)
            points = self.lidar.get_lidar_points(cached=cached)
            if len(points) == 0:
                continue

            if cb is not None:
                cb(points, only_nearby_meters=only_nearby_meters)

    def show_lidar_cloud(self, image, **kwargs):
        self._auto_update_lidar_cloud(
            cb=lambda points, only_nearby_meters: self._update_picture(
                image, points, only_nearby_meters=only_nearby_meters
            ),
            **kwargs
        )
