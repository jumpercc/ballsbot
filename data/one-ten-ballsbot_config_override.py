import math

LIDAR_CALIBRATION = {
    'angle_fix': 3.0660432755299887,
    'fl_x': 0.3443406823910605,
    'fl_y': 0.15713610488972193,
    'rr_x': -0.07267960972661196,
    'rr_y': -0.15285037016562764,
}
LIDAR_CALIBRATION_WITHOUT_MANIPULATOR = {
    'angle_fix': 3.0723390524559697,
    'fl_x': 0.2731797383269527,
    'fl_y': 0.1573384852164638,
    'rr_x': -0.0759998125850887,
    'rr_y': -0.15234530022269227,
}
LIDAR_CALIBRATION_RANGE_LIMIT = 0.5
FROM_LIDAR_TO_CENTER = 0.203

TURN_DIAMETER = 1.43

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

DISTANCE_SENSORS = {
    "front": {
        "offset": 275,  # mm from lidar center
        "direction": "forward",
    },
    "rear": {
        "offset": 80,  # mm from lidar center
        "direction": "backward",
    },
    "manipulator": {
        "offset": 325,  # mm from lidar center
        "direction": "forward",
    },
}
DISTANCE_SENSORS_MESSAGE_TYPE = "ballsbot_tca9548"

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
