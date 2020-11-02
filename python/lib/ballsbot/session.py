import sys

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
import rospy


def start_rospy_session(my_name):
    rospy.init_node(my_name)
