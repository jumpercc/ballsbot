# result of Lidar.calibrate()
LIDAR_CALIBRATION = {
    'angle_fix': 3.0810609316737434,  # lidar orientation (radians)
    'fl_x': 0.2041686441507201,  # front left corner coords
    'fl_y': 0.10426277741236079,
    'rr_x': -0.08011094659163859,  # rear right corner coords
    'rr_y': -0.0988675829003773,
}

# meters per count of primary or secondary peaks
ODOMETRY_METERS_PER_ROTATION = 3.9 / 47.

# ODOMETRY_PRIMARY_PIN = 18  # BOARD pin 12 T200 default
ODOMETRY_PRIMARY_PIN = 20  # BOARD pin 38 T208 default

ODOMETRY_SECONDARY_PIN = 19  # BOARD pin 35

PCA9685_I2C_BUSNUM = 0  # for T208 and pins 27, 28
# PCA9685_I2C_BUSNUM = 1  # for T200 and pins 3, 5

STEERING_CHANNEL = 0
THROTTLE_CHANNEL = 1

# steering
SERVOS_LEFT_PULSE = 450
SERVOS_RIGHT_PULSE = 330

# throttle
SERVOS_MIN_PULSE = 328
SERVOS_MAX_PULSE = 390
SERVOS_ZERO_PULSE = 360

LASER_SENSOR_FRONT_ENABLED = True
LASER_SENSOR_REAR_ENABLED = True
