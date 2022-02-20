import ipywidgets.widgets as widgets
import sys

sys.path.append('/home/ballsbot/projects/ballsbot/python/lib')
import ballsbot.config as config

config.ENABLE_MULTIPROCESSING = False

from ballsbot.utils import keep_rps, join_all_theads, _all_threads
from ballsbot.augmented_lidar import get_augmented_lidar
from ballsbot.ros_messages import get_ros_messages
from ballsbot.manipulator import Manipulator
from ballsbot.joystick import JoystickWrapper
from ballsbot.drawing import ManipulatorPoseDrawing, BotPoseSelfCoordsDrawing, StatusMessagesDrawing
from ballsbot.dashboard import Dashboard
from ballsbot.lidar_drawing import LidarDrawing
from ballsbot.ups import UPS


def main():
    ros = get_ros_messages()
    ros.start(sync=False)
    lidar = get_augmented_lidar()
    lidar.start()

    ups = UPS()

    dashboard = Dashboard(widgets)

    lidar_drawing_raw = BotPoseSelfCoordsDrawing(dashboard, 'lidar')
    lidar_drawing = LidarDrawing(lidar, lidar_drawing_raw)
    dashboard.add_subplot('lidar', lidar_drawing_raw.get_drawing_func_name())

    status_drawing_raw = StatusMessagesDrawing(dashboard, 'status')
    dashboard.add_subplot('status', status_drawing_raw.get_drawing_func_name())

    manipulator_drawing_raw = ManipulatorPoseDrawing(dashboard, 'manipulator_xy', 'manipulator_xz')
    dashboard.add_subplot('manipulator_xy', manipulator_drawing_raw.get_drawing_func_name())
    dashboard.add_subplot('manipulator_xz', manipulator_drawing_raw.get_drawing_func_name())

    an_image = dashboard.get_image()

    controller = widgets.Controller(index=0)
    joystick_wrapper = JoystickWrapper(controller)
    manipulator = Manipulator(joystick_wrapper, without_encoders=True)

    ts = None
    for _ in range(30):
        ts = keep_rps(ts, fps=1)

        lidar_drawing.update_image_once(cached=False)

        capacity = int(round(ups.get_capacity() or -1.))
        distances = lidar.cached_distances
        if distances and 'manipulator' in distances and distances['manipulator']:
            distance = distances['manipulator'].get('distance', -1)
        else:
            distance = -1
        status_drawing_raw.update_image(f'battery charge: {capacity}%\ntarget: {distance} m')

        pose = manipulator.get_manipulator_pose()
        manipulator_drawing_raw.update_image(pose, override_crop_half_size=750.)

        dashboard.redraw()

    ros.stop()
    joystick_wrapper.stop()
    manipulator.stop()

    ts = None
    while True:
        ts = keep_rps(ts, fps=1)
        if all(x.is_alive() for x in _all_threads):
            break
        join_all_theads()


if __name__ == '__main__':
    main()
