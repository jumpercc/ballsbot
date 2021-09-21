from Jetson import GPIO
import sys
sys.path.append('/home/ballsbot/projects/ballsbot/lib')

from ballsbot.utils import keep_rps
from ballsbot.config import ODOMETRY_PRIMARY_PIN, ODOMETRY_SECONDARY_PIN


def main():
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi

    GPIO.setup(ODOMETRY_PRIMARY_PIN, GPIO.IN)
    GPIO.setup(ODOMETRY_SECONDARY_PIN, GPIO.IN)

    def primary_both(_):
        if GPIO.input(ODOMETRY_PRIMARY_PIN) == GPIO.HIGH:
            print('primary high')
        else:
            print('primary low')

    def secondary_both(_):
        if GPIO.input(ODOMETRY_SECONDARY_PIN) == GPIO.HIGH:
            print('secondary high')
        else:
            print('secondary low')

    try:
        GPIO.add_event_detect(ODOMETRY_PRIMARY_PIN, GPIO.BOTH, callback=primary_both)
        GPIO.add_event_detect(ODOMETRY_SECONDARY_PIN, GPIO.BOTH, callback=secondary_both)

        ts = None
        while True:
            ts = keep_rps(ts, fps=2)

    finally:
        GPIO.remove_event_detect(ODOMETRY_PRIMARY_PIN)
        GPIO.remove_event_detect(ODOMETRY_SECONDARY_PIN)
        GPIO.cleanup()


if __name__ == '__main__':
    main()
