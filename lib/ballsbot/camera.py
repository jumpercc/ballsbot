import cv2
import ipywidgets.widgets as widgets
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import atexit
import threading
import numpy as np

from ballsbot.utils import keep_rps, bgr8_to_jpeg


def get_captures(cameras_count):
    if cameras_count <= 0:
        raise ValueError('no cameras specified')
    caps = []
    for cam_id in range(0, 3):
        cap = cv2.VideoCapture(cam_id)
        if not cap.isOpened():
            print('Error opening input video: {}'.format(cam_id))
            continue
        # print('{} opened'.format(cam_id))
        caps.append(cap)
        if len(caps) == cameras_count:
            break
    return caps


class Camera(SingletonConfigurable):
    value = traitlets.Any()

    def __init__(self, cap, capture_size):
        super(Camera, self).__init__()
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, capture_size['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_size['height'])
        self.value = np.empty((capture_size['height'], capture_size['width'], 3), dtype=np.uint8)
        self.cap = cap
        self.thread = threading.Thread(target=self._capture_frames)
        self.thread.start()
        atexit.register(self.stop)

    def _capture_frames(self):
        ts = None
        while True:
            ts = keep_rps(ts, fps=25)

            ret, frame = self.cap.read()
            if not ret:
                break
            self.value = frame

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'thread'):
            self.thread.join()


def get_images_and_cameras(image_width=640, image_height=360):
    capture_sizes = [  # FIXME
        {'width': 1280, 'height': 720},
        {'width': 640, 'height': 360},
    ]

    images = []
    for _ in range(len(capture_sizes)):
        image = widgets.Image(format='jpeg', width=image_width, height=image_height)
        images.append(image)
    sidebyside = widgets.HBox(images)

    caps = get_captures(len(capture_sizes))
    cameras = []
    links = []
    for i, image in enumerate(images):
        cameras.append(Camera(caps[i], capture_sizes[i]))
        links.append(
            traitlets.dlink(
                (cameras[-1], 'value'),
                (image, 'value'),
                transform=lambda x: bgr8_to_jpeg(cv2.resize(x, (image_width, image_height)))
            )
        )
    return sidebyside, cameras
