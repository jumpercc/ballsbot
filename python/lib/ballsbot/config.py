import os
import os.path
import sys

# result of Lidar.calibrate()
LIDAR_CALIBRATION_WITHOUT_MANIPULATOR = {
    'angle_fix': 3.0810609316737434,  # lidar orientation (radians)
    'fl_x': 0.2041686441507201,  # front left corner coords
    'fl_y': 0.10426277741236079,
    'rr_x': -0.08011094659163859,  # rear right corner coords
    'rr_y': -0.0988675829003773,
}
LIDAR_CALIBRATION = LIDAR_CALIBRATION_WITHOUT_MANIPULATOR
LIDAR_CALIBRATION_RANGE_LIMIT = 0.3
TURN_DIAMETER = 0.88
FROM_LIDAR_TO_CENTER = 0.07

# meters per count of primary or secondary peaks
ODOMETRY_METERS_PER_ROTATION = 3.9 / 47.

# ODOMETRY_PRIMARY_PIN = 18  # BOARD pin 12 T200 default
ODOMETRY_PRIMARY_PIN = 20  # BOARD pin 38 T208 default

ODOMETRY_SECONDARY_PIN = 19  # BOARD pin 35

PCA9685_I2C_BUSNUM = 0  # for T208 and pins 27, 28
# PCA9685_I2C_BUSNUM = 1  # for T200 and pins 3, 5
PCA9685_I2C_ADDR = 0x40  # 9865, over rides only if needed, ie. TX2..

CAR_CONTROLS = {
    'steering': {
        "channel": 0,
        "min_pulse": 450,
        "max_pulse": 330,
        "default_position": 0.,
        "control": {"axis": 0},
    },
    'throttle': {
        "channel": 1,
        "min_pulse": 330,
        "zero_pulse": 360,
        "max_pulse": 390,
        "default_position": 0.,
        "control": {"axis": 1, "reverse": True},
        "turbo_control": {"button": 1},
    },
}

LASER_SENSOR_FRONT_ENABLED = True
LASER_SENSOR_FRONT_OFFSET = 190  # mm from lidar center
LASER_SENSOR_REAR_ENABLED = True
LASER_SENSOR_REAR_OFFSET = 60  # mm from lidar center

MANIPULATOR = {
    "enabled": False,
    "has_camera": False,
}
SECONDS_TO_UNFOLD_MANIPULATOR = 6

ENABLE_MULTIPROCESSING = True

if os.path.isfile(os.environ['HOME'] + '/ballsbot_config_override.py'):
    sys.path.append(os.environ['HOME'])
    # noinspection PyUnresolvedReferences
    from ballsbot_config_override import *  # pylint: disable=E0401, W0401
    if sys.path[-1] == os.environ['HOME']:
        sys.path.pop()

CAR_WIDTH = 2. * max(abs(LIDAR_CALIBRATION['fl_y']), abs(LIDAR_CALIBRATION['rr_y']))

if 'min_turbo_pulse' not in CAR_CONTROLS['throttle']:
    CAR_CONTROLS['throttle']['min_turbo_pulse'] = CAR_CONTROLS['throttle']['min_pulse']
if 'max_turbo_pulse' not in CAR_CONTROLS['throttle']:
    CAR_CONTROLS['throttle']['max_turbo_pulse'] = CAR_CONTROLS['throttle']['max_pulse']
