from ballsbot.utils import keep_rps
from ballsbot.lidar import calibration_to_xywh


class LidarDrawing:
    def __init__(self, lidar, dash_callback):
        self.lidar = lidar
        self.position = calibration_to_xywh(lidar.get_calibration())
        self.dash_callback = dash_callback

    def update_image_once(self, cached=True, only_nearby_meters=4):
        points = self.lidar.get_lidar_points(cached=cached)
        if len(points) == 0:
            return

        self.dash_callback.update_image(points, self.position, only_nearby_meters, None)

    def auto_update_image(self, fps=1, **kwargs):
        ts = None
        while True:
            ts = keep_rps(ts, fps=fps)
            self.update_image_once(**kwargs)
