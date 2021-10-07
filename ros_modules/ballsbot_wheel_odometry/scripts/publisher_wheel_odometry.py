#!/usr/bin/env python3
from time import time
import sys
from Jetson import GPIO

sys.path.append('/home/ballsbot/projects/ballsbot/python/lib')

from ballsbot.config import ODOMETRY_METERS_PER_ROTATION, ODOMETRY_PRIMARY_PIN, ODOMETRY_SECONDARY_PIN

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/home/ballsbot/catkin_ws/devel/lib/python2.7/dist-packages')

import rospy
from ballsbot_wheel_odometry.msg import OdometryState


class Odometry:
    def __init__(self):
        self.teta_speed = 0.08

        self.odometry_counter = 0
        self.direction = 0.
        self.odometry_last_interval = None

        self.event_timestamps = {
            'primary_rising': None,
            'primary_falling': None,
            'secondary_rising': None,
            'secondary_falling': None,
            'any_event': None,
        }

    def get_speed(self):
        if self.odometry_last_interval is not None:
            return ODOMETRY_METERS_PER_ROTATION / self.odometry_last_interval
        return 0.

    def get_direction(self):
        return self.direction


def publisher():
    odometry = Odometry()

    pub = rospy.Publisher('wheel_odometry', OdometryState, queue_size=2)
    rospy.init_node('ballsbot_wheel_odometry')
    rate = rospy.Rate(4)

    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    GPIO.setup(ODOMETRY_PRIMARY_PIN, GPIO.IN)
    GPIO.setup(ODOMETRY_SECONDARY_PIN, GPIO.IN)

    detections = odometry.event_timestamps

    def primary_both(_):
        if GPIO.input(ODOMETRY_PRIMARY_PIN) == GPIO.HIGH:
            detections['primary_rising'] = time()
            detections['any_event'] = detections['primary_rising']
        else:
            ts = time()
            if detections['primary_falling'] is not None:
                prev_ts = detections['primary_falling']
                an_interval = ts - prev_ts
                if 0.05 < an_interval < 2.:
                    odometry.odometry_last_interval = an_interval
            detections['primary_falling'] = ts
            detections['any_event'] = detections['primary_falling']

    def secondary_both(_):
        if GPIO.input(ODOMETRY_SECONDARY_PIN) == GPIO.HIGH:
            detections['secondary_rising'] = time()
            detections['any_event'] = detections['secondary_rising']
            odometry.odometry_counter += 1
        else:
            detections['secondary_falling'] = time()
            detections['any_event'] = detections['secondary_falling']

    try:
        GPIO.add_event_detect(ODOMETRY_PRIMARY_PIN, GPIO.BOTH, callback=primary_both)
        GPIO.add_event_detect(ODOMETRY_SECONDARY_PIN, GPIO.BOTH, callback=secondary_both)

        while not rospy.is_shutdown():
            if detections['any_event'] is None or time() - detections['any_event'] > 0.5:
                # reset state if no events (car stopped)
                odometry.odometry_last_interval = None
                odometry.direction = 0.
                for k in detections:
                    detections[k] = None
            elif detections['primary_falling'] is not None and detections['secondary_rising'] is not None \
                    and (detections['primary_rising'] is not None or detections['secondary_falling'] is not None):
                # select events to determine direction
                select_rising = False
                if detections['primary_rising'] is not None and detections['secondary_falling'] is not None:
                    select_rising = bool(
                        abs(detections['primary_rising'] - detections['secondary_rising']) <
                        abs(detections['primary_falling'] - detections['secondary_falling'])
                    )
                # determine direction
                if select_rising and detections['primary_rising'] is not None:
                    odometry.direction = 1. if detections['primary_rising'] < detections['secondary_rising'] else -1.
                else:
                    odometry.direction = (
                        1. if detections.get('primary_falling', 0.) < detections.get('secondary_falling', 0.) else -1.)

            rate.last_time = rospy.rostime.get_rostime()
            message = OdometryState()
            message.speed = odometry.get_speed()
            message.direction = odometry.get_direction()
            message.odometry_counter = odometry.odometry_counter
            rospy.loginfo(message.speed)
            pub.publish(message)

            rate.sleep()

    finally:
        GPIO.remove_event_detect(ODOMETRY_PRIMARY_PIN)
        GPIO.remove_event_detect(ODOMETRY_SECONDARY_PIN)
        GPIO.cleanup()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
