import argparse
import sys
import logging
import json
import os.path
from time import sleep, time
from typing import Optional
import http.server
import socketserver
from urllib.parse import parse_qs
import uuid
import ssl
import numpy as np
import cv2

logging.basicConfig(
    format='[%(asctime)s] %(levelname)s %(name)s: %(message).700s',
    stream=sys.stderr,
    level=logging.INFO
)
logger = logging.getLogger(__name__)

sys.path.append('/home/ballsbot/projects/ballsbot/python/lib')

from ballsbot.utils import run_as_thread, join_all_threads, keep_rps, bgr8_to_jpeg
from ballsbot.config import MANIPULATOR, T208_UPS, DISTANCE_SENSORS
from ballsbot.controller import link_controller, stop_controller
from ballsbot.lidar import revert_transformation_to_cloud
from ballsbot.lidar_with_memory import LidarWithMemory
from ballsbot.ros_messages import get_ros_messages
from ballsbot.manipulator import Manipulator
from ballsbot.joystick import JoystickWrapperBaseWithUpdates, JoystickMock
from ballsbot.ups import UPS
from ballsbot.ros_cameras import Cameras
from ballsbot.pose import Pose
from ballsbot.lidar import calibration_to_xywh
from ballsbot.detection import Detector
from ballsbot.ai.explorer import Explorer
from ballsbot.manipulator_runner import ManipulatorRunner
from ballsbot.poses_generators import DetectorPoses
from ballsbot.tracking import Tracker

server_config_dir = '/home/ballsbot/web_server_config'
expected_password = None
expected_token = None
bot: Optional['TeleoperationBot'] = None


class TeleoperationBot:
    def __init__(self, small_images):
        self.lidar = None
        self.ups = None
        self.joystick_mock = JoystickMock()
        self.joystick = JoystickWrapperBaseWithUpdates(self.joystick_mock)
        if MANIPULATOR.get('enabled'):
            self.manipulator = Manipulator(self.joystick)
        else:
            self.manipulator = None
        self.tracker = None
        self.running = False
        self.cameras = None
        self.pose = None
        self.joystick_state_updated_at = None
        self.fps = 4
        self.camera_dimensions = [
            (
                {'width': 320, 'height': 240}
                if small_images else
                {'width': 640, 'height': 480}
            ),
            {'width': 320, 'height': 240},
        ]
        self.detector = Detector()
        self.bot_mode = "manual"
        self.current_bot = None

    def run(self):
        self.running = True
        self.pose = Pose()
        self.lidar = LidarWithMemory()
        self.tracker = Tracker(self.lidar, self.pose)
        self.tracker.start()

        if T208_UPS:
            self.ups = UPS()

        link_controller(self.joystick)

        self.cameras = Cameras()
        assert self.cameras.count() <= len(self.camera_dimensions)

        ts = None
        while self.running:
            ts = keep_rps(ts, fps=self.fps)

            if self.joystick_state_updated_at and time() - self.joystick_state_updated_at >= 2.:
                logger.warning('connection lost, no updates from controller')
                self.joystick_mock.stop()
                self.joystick.update_all()
                self.joystick_state_updated_at = None

    def stop(self):
        self.running = False

        if self.current_bot:
            self.current_bot.stop()
            self.current_bot = None

        self.joystick.stop()
        sleep(0.5)

        stop_controller()
        if self.manipulator:
            self.manipulator.stop()

        self.tracker.stop()

    def update_joystick_state(self, axes, buttons, mode):
        if self.bot_mode != mode:
            self.joystick_mock.stop()
            self.joystick.update_all()

        self.bot_mode = mode

        self.joystick_mock.set_values(axes_values=axes, buttons_values=buttons)
        if mode == 'manual':
            if self.current_bot:
                self.current_bot.stop()
                self.current_bot = None
                join_all_threads()
            self.joystick.update_all()
        elif mode in {'claw-detector', 'explorer'}:
            if not self.current_bot:
                self.current_bot = self._get_bot_for(mode)
                run_as_thread(lambda: self.current_bot.run())
            if mode == 'claw-detector':
                self.joystick.update_car_controls()
        else:
            logger.warning('unknown bot mode %s', mode)
            self.bot_mode = 'manual'

        self.joystick_state_updated_at = time()

    def get_state(self):
        pose = self.pose.get_pose()
        tracker_params = self.tracker.get_picture_params(with_free_tiles=True)
        if tracker_params:
            _, _, _, free_cells, target_point, _ = tracker_params
            cells = [target_point] + free_cells
            transformation = (pose['x'], pose['y'], pose['teta'])
            transformed_cells = revert_transformation_to_cloud(cells, transformation)
            free_cells = transformed_cells[1:]
            target_point = transformed_cells[0]
        else:
            free_cells = []
            target_point = [0., 0.]
        return {
            'lidar': self.lidar.lidar.get_lidar_points(cached=False),
            'distance_sensors': self.lidar.lidar.cached_distances,  # must be after get_lidar_points
            'pose': pose,
            'bot_size': calibration_to_xywh(self.lidar.lidar.get_calibration()),
            'ups': (self.ups.get_capacity() if self.ups else None),
            'manipulator': (self.manipulator.get_track_frame() if self.manipulator else None),
            'detections': self.detector.get_detections(),
            'current_mode': self.bot_mode,
            'modes_available': [
                                   'manual',
                                   'explorer',
                               ] + (
                                   ['claw-detector'] if self.manipulator else []
                               ),
            'free_tile_centers': free_cells,
            'target_point': target_point,
        }

    def get_image(self, index):
        raw_images = self.cameras.get_images()
        if index in raw_images:
            raw_rgb_bytes = np.asarray(
                bytearray(raw_images[index].image), dtype=np.uint8
            ).reshape(
                (raw_images[index].image_height, raw_images[index].image_width, 3)
            )
            if (
                    raw_images[index].image_height == self.camera_dimensions[index]['height'] and
                    raw_images[index].image_width == self.camera_dimensions[index]['width']
            ):
                raw_value = raw_rgb_bytes
            else:
                raw_value = cv2.resize(
                    raw_rgb_bytes,
                    (self.camera_dimensions[index]['width'], self.camera_dimensions[index]['height']),
                    0, 0,
                    cv2.INTER_AREA
                )
        else:
            raw_value = np.empty(
                (self.camera_dimensions[index]['height'], self.camera_dimensions[index]['width'], 3),
                dtype=np.uint8
            )

        return bgr8_to_jpeg(raw_value)

    def get_settings(self):
        return {
            'cameras': (
                    ['front'] +
                    list(
                        ['manipulator']
                        if (MANIPULATOR.get('enabled') and MANIPULATOR.get('has_camera'))
                        else ()
                    )
            ),
            'distance_sensors': list(sorted(DISTANCE_SENSORS.keys())),
            'lidar': True,
            'manipulator': MANIPULATOR.get('enabled'),
            'ups': bool(T208_UPS),
            'pose': True,
            'updates_per_second': self.fps,
            'camera_image': self.camera_dimensions[0:self.cameras.count()],
        }

    def _get_bot_for(self, mode):
        if mode == 'claw-detector':
            poses_generator = DetectorPoses(
                detector=self.detector,
                already_running=True,
            )
            return ManipulatorRunner(
                poses_generator,
                joystick=self.joystick,
                manipulator=self.manipulator,
                already_running=True,
            )
        elif mode == 'explorer':
            return Explorer(
                lidar_with_memory=self.lidar,
                tracker=self.tracker,
                detector=self.detector,
                joystick=self.joystick,
                already_running=True,
            )
        else:
            raise NotImplementedError(mode)


class WebServerHandler(http.server.BaseHTTPRequestHandler):
    close_connection = False
    protocol_version = 'HTTP/1.1'

    ACTIONS = {
        ('POST', '/auth'): 'handle_auth',
        ('GET', '/settings'): 'handle_settings',
        ('GET', '/camera_image'): 'handle_camera_image',
        ('POST', '/controller_state'): 'handle_controller_state',
    }

    @staticmethod
    def handle_auth(parameters):
        got_key = parameters.get('key')
        if not got_key:
            return None
        global expected_password, expected_token
        if expected_password is None:
            file_name = os.path.join(server_config_dir, 'password')
            logger.debug('opening password file %s', file_name)
            with open(file_name) as hf:
                expected_password = hf.readline().rstrip()
        if got_key != expected_password:
            return None
        expected_token = uuid.uuid1().hex
        return {'token': expected_token}

    @staticmethod
    def is_token_valid(parameters):
        global expected_token
        got_token = parameters.get('token')
        logger.debug("got %s, expected %s", got_token, expected_token)
        return (
                got_token and
                got_token == expected_token
        )

    @staticmethod
    def handle_settings(_):
        global bot
        return bot.get_settings()

    @staticmethod
    def handle_camera_image(parameters):
        global bot
        image_index = int(parameters.get('image_index'))
        return bot.get_image(image_index)

    @staticmethod
    def handle_controller_state(parameters):
        global bot
        mode = parameters.get('mode')
        controller_state = json.loads(parameters.get('controller_state'))
        bot.update_joystick_state(axes=controller_state['axes'], buttons=controller_state['buttons'], mode=mode)
        return bot.get_state()

    def generic_handle(self, http_method):
        try:
            parts = self.path.split('?', 2)
            if len(parts) == 2:
                path, query_string = parts
            else:
                path = parts[0]
                query_string = None
            logger.debug('%s %s', http_method, path)

            method_name = self.ACTIONS.get((http_method, path))
            if method_name:
                if http_method == 'POST':
                    length = int(self.headers.get('content-length'))
                    field_data = self.rfile.read(length)
                    parameters = my_parse_qs(field_data)
                else:
                    parameters = {}
                if query_string:
                    parameters.update(my_parse_qs(query_string))
                logger.debug('parameters: %s', parameters)

                if method_name != 'handle_auth' and not self.is_token_valid(parameters):
                    self.show_error(403, b'Wrong token')
                    return

                try:
                    content_struct = getattr(self, method_name)(parameters)
                except Exception as e:
                    logger.error('failed with %s', e)
                    self.show_error(500, b'Server error')
                    return

                if method_name == 'handle_auth' and not content_struct:
                    self.show_error(403, b'Wrong key')
                    return

                is_image_response = (method_name == 'handle_camera_image')
                if not is_image_response:
                    logger.debug('response: %s', content_struct)

                self.send_response(200)
                self.send_header('Cache-Control', 'no-store')

                if is_image_response:
                    self.send_header('Content-type', 'image/jpeg')
                    content = content_struct
                else:
                    self.send_header('Content-type', 'application/json')
                    content = json.dumps(content_struct).encode()

                self.send_header('Content-Length', str(len(content)))
                self.end_headers()

                self.wfile.write(content)
            else:
                self.show_error(404, b'Not found')
            self.wfile.flush()
        except ConnectionResetError:
            logging.warning('Connection reset by peer')

    def show_error(self, http_code, content, content_type='text/plain'):
        self.send_response(http_code)
        self.send_header('Content-type', content_type)
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self):
        self.generic_handle('GET')

    def do_POST(self):
        self.generic_handle('POST')

    def log_message(self, format, *args):
        return  # disable logging


def my_parse_qs(qs):
    result = parse_qs(qs)
    return {
        (k.decode('utf-8') if isinstance(k, bytes) else k):
            (v[0].decode('utf-8') if isinstance(v[0], bytes) else v[0])
        for k, v in result.items()
    }


class WebServer:
    def __init__(self, config_dir, args):
        self.config_dir = config_dir
        self.ip = args.ip
        self.port = args.port
        self.https = args.https
        self.httpd = None

    def start(self):
        self.httpd = socketserver.TCPServer((self.ip, self.port), WebServerHandler, bind_and_activate=False)
        self.httpd.allow_reuse_address = True
        self.httpd.server_bind()
        self.httpd.server_activate()

        if self.https:
            # openssl req -x509 -nodes -days 730 -newkey rsa:2048 -keyout \
            # certificate_key -out certificate -config cert.config
            self.httpd.socket = ssl.wrap_socket(
                self.httpd.socket,
                server_side=True,
                certfile=os.path.join(self.config_dir, 'certificate'),
                keyfile=os.path.join(self.config_dir, 'certificate_key'),
                ssl_version=ssl.PROTOCOL_TLS
            )

        logger.info('HTTP%s server on %s:%s', ('S' if self.https else ''), self.ip, self.port)
        self.httpd.serve_forever()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        logger.info('http server stopped')


def main():
    global server_config_dir, bot
    parser = argparse.ArgumentParser(description='Start ballsbot teleoperation.')
    parser.add_argument('--ip', dest='ip', help='IP address', default='0.0.0.0')
    parser.add_argument('--port', dest='port', help='Port number', default=4443, type=int)
    parser.add_argument('--https', dest='https', help='HTTPS', default=True, type=int)
    parser.add_argument('--server-config-dir', dest='server_config_dir',
                        help='directory with password, cert and (cert) key', default=server_config_dir)
    args = parser.parse_args()
    server_config_dir = args.server_config_dir

    web_server = WebServer(server_config_dir, args)
    run_as_thread(web_server.start)

    bot = TeleoperationBot(
        small_images=(args.ip == '127.0.0.1'),
    )
    run_as_thread(bot.run)

    try:
        get_ros_messages().start(sync=True)
    except KeyboardInterrupt:
        pass
    web_server.stop()
    bot.stop()
    join_all_threads()


if __name__ == '__main__':
    main()
