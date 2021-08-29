import math

LIDAR_CALIBRATION = {  # FIXME
    'angle_fix': 3.083771826820918,
    'fl_x': 0.3371084301852129,
    'fl_y': 0.16540299714227663,
    'rr_x': -0.1044873414628473,
    'rr_y': -0.1589888544588804
}
LIDAR_CALIBRATION_WITHOUT_MANIPULATOR = {  # FIXME
    'angle_fix': 3.083771826820918,
    'fl_x': 0.29,
    'fl_y': 0.16540299714227663,
    'rr_x': -0.1044873414628473,
    'rr_y': -0.1589888544588804
}
LIDAR_CALIBRATION_RANGE_LIMIT = 0.5
FROM_LIDAR_TO_CENTER = 0.203

TURN_DIAMETER = 1.25  # FIXME

# meters per count of primary or secondary peaks
# ODOMETRY_METERS_PER_ROTATION = 3.9 / 47.

CAR_CONTROLS = {
    'steering': {
        "channel": 0,
        "min_pulse": 470,
        "max_pulse": 320,
        "default_position": 0.,
        "control": {"axis": 0},
    },
    'throttle': {
        "channel": 1,
        "min_turbo_pulse": 344,
        "min_pulse": 354,  # 150
        "zero_pulse": 379,  # 375
        "max_pulse": 394,  # 600
        "max_turbo_pulse": 404,
        "default_position": 0.,
        "control": {"axis": 1, "reverse": True},
        "turbo_control": {"button": 1},
    },
}

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
            "min_pulse": 453,
            "max_pulse": 306,
            "default_position": 0.9,
            "unfold_position": 0.,
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
    "fold": {
        "button_fold": 0,
        "button_unfold": 3,
        "button_stop": 2,
    },
}
