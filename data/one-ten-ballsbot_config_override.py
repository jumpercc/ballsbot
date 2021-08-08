import math

LIDAR_CALIBRATION = {
    'angle_fix': 3.083771826820918,
    'fl_x': 0.3371084301852129,
    'fl_y': 0.16540299714227663,
    'rr_x': -0.1044873414628473,
    'rr_y': -0.1589888544588804
}
LIDAR_CALIBRATION_WITHOUT_MANIPULATOR = {
    'angle_fix': 3.083771826820918,
    'fl_x': 0.29,
    'fl_y': 0.16540299714227663,
    'rr_x': -0.1044873414628473,
    'rr_y': -0.1589888544588804
}
LIDAR_CALIBRATION_RANGE_LIMIT = 0.5
FROM_LIDAR_TO_CENTER = 0.203

TURN_DIAMETER = 1.25

# meters per count of primary or secondary peaks
#ODOMETRY_METERS_PER_ROTATION = 3.9 / 47.

# steering
SERVOS_LEFT_PULSE = 470
SERVOS_RIGHT_PULSE = 320

# throttle
SERVOS_MIN_PULSE = 324  # 150
SERVOS_ZERO_PULSE = 393  # 375
SERVOS_MAX_PULSE = 424  # 600

LASER_SENSOR_FRONT_ENABLED = False
LASER_SENSOR_FRONT_OFFSET = 275  # mm from lidar center
LASER_SENSOR_REAR_ENABLED = False
LASER_SENSOR_REAR_OFFSET = 80  # mm from lidar center

MANIPULATOR = {
    "enabled": True,
    "has_camera": True,
    "servos": [
        {
            "channel": 4,
            "min_pulse": 477,
            "max_pulse": 234,
            "default_position": 0.,
            "unfold_position": 0.,
            "min_angle": math.radians(-60.),
            "max_angle": math.radians(60.),
            "control": {"axis": 2},
        },
        {
            "channel": 5,
            "min_pulse": 216,
            "max_pulse": 533,
            "default_position": -1.,
            "unfold_position": 0.5,
            "min_angle": math.radians(-85.),
            "max_angle": math.radians(90.),
            "control": {"buttons": (7, 5)},
        },
        {
            "channel": 6,
            "min_pulse": 197,
            "max_pulse": 530,
            "default_position": 1.,
            "unfold_position": -0.7,
            "min_angle": math.radians(-90.),
            "max_angle": math.radians(90.),
            "control": {"axis": 3},
        },
        {
            "channel": 7,
            "min_pulse": 591,
            "max_pulse": 393,
            "default_position": 1.,
            "unfold_position": -0.5,
            "min_angle": math.radians(0.),
            "max_angle": math.radians(150.),
            "control": {"buttons": (6, 4)},
        },
    ],
    "bones": [
        {
            "start": True,
            "default_position": (195., 0., -105.),
        },
        {
            "rotation": "z",
            "direction": -1.,
            "default_position": (35., 0., 45.),
        },
        {
            "rotation": "y",
            "direction": -1.,
            "default_position": (-165., 0., 5.),
        },
        {
            "rotation": "y",
            "direction": -1.,
            "default_position": (240., 0., -5.),
        },
        {
            "default_position": (55., 0., 0.),
            "rotation": "z",
            "direction": 1.,
            "claw": True,
        },
    ],
}