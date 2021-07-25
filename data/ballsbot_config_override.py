import math

LIDAR_CALIBRATION = {
    'angle_fix': 3.1237764080339154,
    'fl_x': 0.23090432880217832,
    'fl_y': 0.1539137515794161,
    'rr_x': -0.08784647285380637,
    'rr_y': -0.14349584614860772,
}

MANIPULATOR = {
    "enabled": True,
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
