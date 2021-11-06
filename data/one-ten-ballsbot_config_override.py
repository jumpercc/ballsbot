import math

T208_UPS = True

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
FROM_LIDAR_TO_PIVOT_CENTER = 0.05

TURN_DIAMETER = 1.43

# meters per count of primary or secondary peaks
ODOMETRY_METERS_PER_ROTATION = 4.67 / 42.

ENGINE_NEED_MANUAL_BREAKING = False

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
    "front-center": {
        "offset_x": 275,  # mm from lidar center
        "offset_y": 0,
        "direction": "forward",
    },
    "front-left": {
        "offset_x": 275,
        "offset_y": 80,
        "direction": "forward",
    },
    "front-right": {
        "offset_x": 275,
        "offset_y": -80,
        "direction": "forward",
    },

    "rear-center": {
        "offset_x": 60,
        "offset_y": 0,
        "direction": "backward",
    },
    "rear-left": {
        "offset_x": 60,
        "offset_y": 80,
        "direction": "backward",
    },
    "rear-right": {
        "offset_x": 60,
        "offset_y": -80,
        "direction": "backward",
    },
    "manipulator": {
        "offset_x": 0,
        "offset_y": 0,
        "direction": "none",
    },
}
DISTANCE_SENSORS_MESSAGE_TYPE = "ballsbot_tca9548"

DETECTION_MAX_DISTANCE_FROM_CENTER_Y = 0.35
DETECTION_CLOSE_ENOUGH = 0.15 * 0.1

MANIPULATOR = {
    "enabled": True,
    "has_camera": True,
    "encoders_enabled": True,
    "servos": [
        {
            "channel": 4,
            "min_pulse": 234,
            "max_pulse": 477,
            "default_position": 0.,
            "unfold_position": 0.,
            "min_angle": 4.417864799499512 - 5.473243236541748,
            "max_angle": 4.417864799499512 - 3.313398599624634,
            "control": {"axis": 2, "invert": True},
            "encoder_name": "m-0",
            "encoder_offset": 4.417864799499512,
        },
        {
            "channel": 5,
            "min_pulse": 216,
            "max_pulse": 533,
            "default_position": -1.,
            "unfold_position": 0.5,
            "min_angle": 3.2888548374176025 - 0.02454369328916073 - math.radians(270.),
            "max_angle": math.radians(90.),
            "control": {"buttons": (7, 5)},
            "encoder_name": "m-1",
            "encoder_offset": math.radians(270.) + 0.02454369328916073,
        },
        {
            "channel": 6,
            "min_pulse": 264,
            "max_pulse": 530,
            "default_position": 1.,
            "unfold_position": -0.7,
            "min_angle": math.radians(90.) - 5.129631519317627 + 2.675262451171875,
            "max_angle": math.radians(90.),
            "control": {"axis": 3},
            "encoder_name": "m-2",
            "encoder_offset": 1.1535537878619593 + math.radians(90.),
            "reverse_encoder": True,
        },
        {
            "channel": 7,
            "min_pulse": 306,
            "max_pulse": 453,
            "default_position": -0.9,
            "unfold_position": 0.,
            "min_angle": 0.,
            "max_angle": 5.718680381774902 - 4.417864799499512,
            "control": {"buttons": (4, 6)},
            "encoder_name": "m-claw",
            "encoder_offset": 4.417864799499512,
        },
    ],
    "bones": [
        {
            "start": True,
            "default_position": (195., 0., -105.),
        },
        {
            "rotation": "z",
            "direction": 1.,
            "default_position": (35., 0., 45.),
        },
        {
            "rotation": "y",
            "direction": -1.,
            "default_position": (5., 0., 165.),
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
