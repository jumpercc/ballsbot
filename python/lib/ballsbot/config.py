import math

# result of Lidar.calibrate()
LIDAR_CALIBRATION_WITHOUT_MANIPULATOR = {
    'angle_fix': 3.0810609316737434,  # lidar orientation (radians)
    'fl_x': 0.2041686441507201,  # front left corner coords
    'fl_y': 0.10426277741236079,
    'rr_x': -0.08011094659163859,  # rear right corner coords
    'rr_y': -0.0988675829003773,
}
LIDAR_CALIBRATION_WITH_MANIPULATOR = {
    'angle_fix': 3.1237764080339154,
    'fl_x': 0.23090432880217832,
    'fl_y': 0.1539137515794161,
    'rr_x': -0.08784647285380637,
    'rr_y': -0.14349584614860772,
}
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

STEERING_CHANNEL = 0
THROTTLE_CHANNEL = 1

# steering
SERVOS_LEFT_PULSE = 450
SERVOS_RIGHT_PULSE = 330

# throttle
SERVOS_MIN_PULSE = 330
SERVOS_MAX_PULSE = 390
SERVOS_ZERO_PULSE = 360

LASER_SENSOR_FRONT_ENABLED = True
LASER_SENSOR_FRONT_OFFSET = 190  # mm from lidar center
LASER_SENSOR_REAR_ENABLED = True
LASER_SENSOR_REAR_OFFSET = 60  # mm from lidar center

MANIPULATOR = {
    "enabled": False,
    "has_camera": False,
    "servos": [
        {
            "channel": 4,
            "min_pulse": 247,
            "max_pulse": 449,
            "default_position": -0.072,
            "min_angle": math.radians(-30.),
            "max_angle": math.radians(30.),
        },
        {
            "channel": 5,
            "min_pulse": 592,
            "max_pulse": 101,
            "default_position": -1.,
            "unfold_position": 0.8,
            "min_angle": math.radians(-90.),
            "max_angle": math.radians(60.),
        },
        {
            "channel": 6,
            "min_pulse": 208,
            "max_pulse": 641,
            "default_position": -1.,
            "unfold_position": 0.8,
            "min_angle": math.radians(-90.),
            "max_angle": math.radians(60.),
        },
        {
            "channel": 7,
            "min_pulse": 185,
            "max_pulse": 423,
            "default_position": -1.,
            "unfold_position": -0.5,
            "min_angle": math.radians(0.),
            "max_angle": math.radians(150.),
        },
    ],
    "bones": [
        {
            "start": True,
            "default_position": (148.5, -100., -56.5),
        },
        {
            "rotation": "y",
            "direction": 1.,
            "default_position": (-19., -5.5, 0.),
        },
        {
            "rotation": "z",
            "direction": 1.,
            "default_position": (-149.5, -19., 0.),
        },
        {
            "rotation": "z",
            "direction": -1.,
            "default_position": (188., -6., 1.),
        },
        {
            "default_position": (55., 0., 0.),
            "rotation": "z",
            "direction": 1.,
            "claw": True,
        },
    ],
}

LIDAR_CALIBRATION = (
    LIDAR_CALIBRATION_WITH_MANIPULATOR if MANIPULATOR['enabled'] else LIDAR_CALIBRATION_WITHOUT_MANIPULATOR
)
CAR_WIDTH = 2. * max(abs(LIDAR_CALIBRATION['fl_y']), abs(LIDAR_CALIBRATION['rr_y']))

ENABLE_MULTIPROCESSING = True
