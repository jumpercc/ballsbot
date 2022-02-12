#!/usr/bin/env python3
import tensorflow as tf
import cv2
import sys
import numpy as np
import threading

sys.path.append('/home/ballsbot/projects/ballsbot/python/lib')

from ballsbot.utils import keep_rps

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/usr/lib/python2.7/dist-packages')
sys.path.append('/home/ballsbot/catkin_ws/devel/lib/python2.7/dist-packages')

import rospy
from ballsbot_detection.msg import DetectionsList, Detection

FRAMERATE = 2
INPUT_TENSOR_SIZE = 320


class Camera:
    def __init__(self, sensor_id=0):
        self.sensor_id = sensor_id
        self.capture_width = 3264
        self.capture_height = 2464
        self.display_width = INPUT_TENSOR_SIZE
        self.display_height = INPUT_TENSOR_SIZE
        self.framerate = FRAMERATE
        self.flip_method = 0

        self.value = np.empty((self.display_height, self.display_width, 3), dtype=np.uint8)
        self.thread = None
        self.cap = None
        self.stopped = False

        try:
            self.cap = cv2.VideoCapture(self._get_gstreamer_str(), cv2.CAP_GSTREAMER)

            re, image = self.cap.read()

            if not re:
                raise RuntimeError('Could not read image from camera.')

            self.value = image
            self.start()
        except:
            self.stop()
            raise RuntimeError('Could not initialize camera.  Please see error trace.')

    def _get_gstreamer_str(self):
        return (
                f"nvarguscamerasrc sensor-id={self.sensor_id} ! video/x-raw(memory:NVMM), " +
                f"width=(int){self.capture_width}, height=(int){self.capture_height}, format=(string)NV12, " +
                f"framerate=(fraction){self.framerate}/1 ! nvvidconv flip-method={self.flip_method} ! " +
                f"video/x-raw, width=(int){self.display_width}, height=(int){self.display_height}, " +
                "format=(string)BGRx ! videoconvert ! appsink"
        )

    def _capture_frames(self):
        while not self.stopped:
            re, image = self.cap.read()
            if re:
                self.value = image
            else:
                break

    def start(self):
        if not self.cap.isOpened():
            self.cap.open(self._get_gstreamer_str(), cv2.CAP_GSTREAMER)
        if not hasattr(self, 'thread') or self.thread is None or not self.thread.isAlive():
            self.thread = threading.Thread(target=self._capture_frames)
            self.thread.start()

    def stop(self):
        self.stopped = True
        if self.cap is not None:
            self.cap.release()
        if self.thread is not None:
            self.thread.join()

    def get_value(self):
        return self.value.copy()


class Detector:
    def __init__(self):
        self.model = tf.saved_model.load(
            '/home/ballsbot/projects/ballsbot/detection_model'
        )
        self.inference_func = self.model.signatures["serving_default"]

        with open('/home/ballsbot/projects/ballsbot/ros_modules/ballsbot_detection/ssd_coco_labels.txt') as hf:
            self.classes = [x.strip() for x in hf]

    def detect(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        rgb_tensor = tf.convert_to_tensor(image, dtype=tf.uint8)
        rgb_tensor = tf.expand_dims(rgb_tensor, 0)

        detection_result = self.inference_func(rgb_tensor)
        scores = detection_result['detection_scores'].numpy()[0]
        result = []
        for i in range(int(detection_result['num_detections'].numpy()[0])):
            if scores[i] < 0.58:
                continue
            class_number = int(detection_result['detection_classes'][0][i].numpy())
            coords = detection_result['detection_boxes'][0][i].numpy()
            y0, x0, y1, x1 = coords
            detection = Detection()
            detection.object_class = self.classes[class_number]
            detection.confidence = scores[i]
            detection.bottom_left.x = x0
            detection.bottom_left.y = 1. - y0
            detection.top_right.x = x1
            detection.top_right.y = 1. - y1
            result.append(detection)

        return result


def publisher():
    rospy.loginfo('loading detector...')
    detector = Detector()
    rospy.loginfo('detector loaded')

    camera = Camera()
    rospy.loginfo('camera started')

    pub = rospy.Publisher('cam_detections', DetectionsList, queue_size=1)
    rospy.init_node('ballsbot_detection')
    rate = rospy.Rate(FRAMERATE)
    rospy.loginfo('start')
    while not rospy.is_shutdown():
        message = DetectionsList()
        for it in detector.detect(camera.get_value()):
            message.data.append(it)
        message.header.stamp = rospy.Time.now()
        pub.publish(message)

        rate.sleep()

    camera.stop()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
