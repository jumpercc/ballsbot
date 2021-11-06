from ballsbot.utils import run_as_thread
from ballsbot.config import DISTANCE_SENSORS, DISTANCE_SENSORS_MESSAGE_TYPE, T208_UPS, MANIPULATOR

from functools import partial
import sys

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/home/ballsbot/catkin_ws/devel/lib/python2.7/dist-packages')
import rospy
from ballsbot_pose.msg import Pose
from ballsbot_detection.msg import DetectionsList
from ballsbot_imu.msg import ImuState
from sensor_msgs.msg import LaserScan
from ballsbot_wheel_odometry.msg import OdometryState
from ballsbot_laser_ranging_sensor.msg import LaserDistance as LaserDistanceStraight
from ballsbot_tca9548.msg import LaserDistance as LaserDistanceSwitch, EncoderAngle
from ballsbot_ups.msg import UpsState

CONFIG_BY_TYPE = {
    "ballsbot_laser_ranging_sensor": {
        "class": LaserDistanceStraight,
        "get_topic": lambda sensor_name: '/laser_distance_' + sensor_name,
    },
    "ballsbot_tca9548": {
        "class": LaserDistanceSwitch,
        "get_topic": lambda _: '/laser_distance',
    },
}


class RosMessages:
    def __init__(self):
        self.subscribe_to = [
            {
                'name': 'pose',
                'topic': '/pose',
                'msg_type': Pose,
            },
            {
                'name': 'imu',
                'topic': '/imu',
                'msg_type': ImuState,
                'for_debug': True,
            },
            {
                'name': 'wheel_odometry',
                'topic': '/wheel_odometry',
                'msg_type': OdometryState,
                'for_debug': True,
            },
            {
                'name': 'cam_detections',
                'topic': '/cam_detections',
                'msg_type': DetectionsList,
            },
            {
                'name': 'lidar',
                'topic': '/scan',
                'msg_type': LaserScan,
            },
        ]

        if DISTANCE_SENSORS_MESSAGE_TYPE == "ballsbot_laser_ranging_sensor":
            for sensor_name in DISTANCE_SENSORS:
                self.subscribe_to.append({
                    'name': 'laser_distance',
                    'topic': CONFIG_BY_TYPE[DISTANCE_SENSORS_MESSAGE_TYPE]["get_topic"](sensor_name),
                    'msg_type': CONFIG_BY_TYPE[DISTANCE_SENSORS_MESSAGE_TYPE]["class"],
                })
        else:
            self.subscribe_to.append({
                'name': 'laser_distance',
                'topic': CONFIG_BY_TYPE[DISTANCE_SENSORS_MESSAGE_TYPE]["get_topic"](None),
                'msg_type': CONFIG_BY_TYPE[DISTANCE_SENSORS_MESSAGE_TYPE]["class"],
            })

        if T208_UPS:
            self.subscribe_to.append({
                'name': 'ups',
                'topic': '/ups',
                'msg_type': UpsState,
            })

        if MANIPULATOR['enabled'] and MANIPULATOR.get('encoders_enabled'):
            self.subscribe_to.append({
                'name': 'magnetic_encoder',
                'topic': '/magnetic_encoder',
                'msg_type': EncoderAngle,
            })

        self.message_by_type = {x['name']: None for x in self.subscribe_to}

    def start(self, sync=True, with_debug_nodes=False):
        rospy.init_node('ballsbot')
        if sync:
            for it in self.subscribe_to:
                if with_debug_nodes or not it.get('for_debug'):
                    rospy.Subscriber(it['topic'], it['msg_type'], partial(self._update_msg_data, it['name']))
            rospy.spin()
        else:
            for it in self.subscribe_to:
                if with_debug_nodes or not it.get('for_debug'):
                    run_as_thread(partial(self._run_node, it))

    def _update_msg_data(self, name, data):
        if name in {'laser_distance', 'magnetic_encoder'}:
            if not self.message_by_type[name]:
                self.message_by_type[name] = {}
            self.message_by_type[name][data.sensor_name] = data
        else:
            self.message_by_type[name] = data

    def _run_node(self, msg_config):
        while True:
            try:
                data = rospy.wait_for_message(msg_config['topic'], msg_config['msg_type'], timeout=5)
            except KeyboardInterrupt:
                return None
            except Exception:  # pylint: disable=W0703
                continue
            if data:
                self._update_msg_data(msg_config['name'], data)

    def list_message_types(self):
        return list(self.message_by_type.keys())

    def get_message_data(self, msg_type):
        if msg_type in {'laser_distance', 'magnetic_encoder'}:
            return (self.message_by_type.get(msg_type) or {}).copy()
        else:
            return self.message_by_type.get(msg_type)


_instance = RosMessages()


def get_ros_messages():
    return _instance
