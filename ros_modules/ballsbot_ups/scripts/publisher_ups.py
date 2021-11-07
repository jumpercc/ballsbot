#!/usr/bin/env python3
import struct
import smbus
import sys

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/home/ballsbot/catkin_ws/devel/lib/python2.7/dist-packages')

import rospy
from ballsbot_ups.msg import UpsState

BUS_NUMBER = 1
DEVICE_ADDRESS = 0x36


def publisher():
    bus = smbus.SMBus(BUS_NUMBER)

    pub = rospy.Publisher('ups', UpsState, queue_size=1)
    rospy.init_node('ballsbot_ups')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        message = UpsState()

        read = bus.read_word_data(DEVICE_ADDRESS, 2)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        message.voltage = swapped * 1.25 / 1000 / 16  # FIXME strange value

        read = bus.read_word_data(DEVICE_ADDRESS, 4)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        message.capacity = swapped / 256

        message.header.stamp = rospy.Time.now()
        pub.publish(message)

        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
