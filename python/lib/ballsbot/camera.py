import cv2
from ipywidgets import widgets
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import numpy as np

from ballsbot.utils import run_as_thread
from ballsbot.utils import keep_rps, bgr8_to_jpeg
from ballsbot.config import MANIPULATOR


class CSICamera(SingletonConfigurable):
    value = traitlets.Any()

    # pylint: disable=R0913
    def __init__(self, capture_width, capture_height, image_width, image_height, fps=5, sensor_id=0):
        super().__init__()
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.image_width = image_width
        self.image_height = image_height
        self.fps = fps
        self.value = np.empty((image_height, image_width, 3), dtype=np.uint8)
        self.sensor_id = sensor_id
        self.cap = None
        run_as_thread(self._capture_frames, self.stop)

    def _get_csi_gsreamer_str(self):
        result = 'nvarguscamerasrc sensor-id=%d ! ' \
                 + 'video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! ' \
                 + 'nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! ' \
                 + 'appsink'
        return result % (
            self.sensor_id,
            self.capture_width,
            self.capture_height,
            self.fps,
            self.image_width,
            self.image_height
        )

    def _capture_frames(self):
        self.cap = cv2.VideoCapture(self._get_csi_gsreamer_str(), cv2.CAP_GSTREAMER)  # pylint: disable=E1101, W0201

        ts = None
        while self.cap is not None:
            ts = keep_rps(ts, fps=self.fps)

            if not self.cap:
                break
            ret, frame = self.cap.read()
            if not ret:
                break
            self.value = frame

    def stop(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def get_cameras(image_width=320, image_height=240, fps=5):
    if MANIPULATOR.get('has_camera'):
        sensors = [0, 1]
    else:
        sensors = [0]

    result = []
    for sensor_number in sensors:
        camera = CSICamera(
            capture_width=3264, capture_height=2464,
            image_width=image_width, image_height=image_height,
            fps=fps, sensor_id=sensor_number
        )
        result.append(camera)

    return result


def get_images_and_cameras(image_width=320, image_height=240, fps=5):
    cameras_list = get_cameras(image_width, image_height, fps)
    result = []
    for camera in cameras_list:
        image = widgets.Image(format='jpeg', width=image_width, height=image_height)
        traitlets.dlink(
            (camera, 'value'),
            (image, 'value'),
            transform=bgr8_to_jpeg,
        )
        result.append((image, camera))

    return result
