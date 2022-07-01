import logging

from ballsbot.config import CAR_DETECTION_MAX_DISTANCE_FROM_CENTER_X, CAR_DETECTION_MAX_DISTANCE_FROM_CENTER_Y
from ballsbot.ros_messages import get_ros_messages

logger = logging.getLogger(__name__)


class Detector:
    def __init__(self):
        self.objects_detected = None
        self.main_object_detected = None
        self.object_classes = {
            'cat': 3,
            'dog': 2,  # cat
            'bear': 2,  # and cat
            'teddy bear': 2,  # and this could be a cat too
            'person': 1,
        }
        self.messenger = get_ros_messages()

    def get_detections(self):
        self.get_seen_object()
        return self.objects_detected.copy() if self.objects_detected else None

    def get_seen_object(self, cached=False):
        if not cached:
            data = self.messenger.get_message_data('cam_detections')
            if data is not None:
                detections = [{
                    'object_class': x.object_class,
                    'top_right': (x.top_right.x, x.top_right.y),
                    'bottom_left': (x.bottom_left.x, x.bottom_left.y),
                    'confidence': x.confidence,
                } for x in data.data]
            else:
                detections = []

            detections = list(filter(lambda x: x['object_class'] in self.object_classes, detections))

            for an_object in detections:
                an_object['hsize'] = an_object['top_right'][0] - an_object['bottom_left'][0]
                an_object['vsize'] = an_object['top_right'][1] - an_object['bottom_left'][1]
                an_object['size'] = an_object['hsize'] * an_object['vsize']
                an_object['center'] = (
                    an_object['bottom_left'][0] + an_object['hsize'] / 2.,
                    an_object['bottom_left'][1] + an_object['vsize'] / 2.,
                )

            # add main object from prev frame if there is no object with this class in current
            objects_detected = detections.copy()
            if self.main_object_detected is not None:
                seen_classes = set(x['object_class'] for x in detections)
                if self.main_object_detected['object_class'] not in seen_classes \
                        and len(list(filter(lambda x: x['object_class'] == self.main_object_detected['object_class'],
                                            self.objects_detected))) != 0:
                    detections.append(self.main_object_detected)

            if len(detections) == 0:
                self.main_object_detected = None
            elif len(detections) == 1:
                self.main_object_detected = detections[0]
            else:
                # first select class by max wight
                # second select largest instance
                self.main_object_detected = list(sorted(
                    detections,
                    key=lambda x: (self.object_classes[x['object_class']], x['size']),
                    reverse=True
                ))[0]

            self.objects_detected = objects_detected

        if self.main_object_detected is None:
            return None
        else:
            return self.main_object_detected.copy()


class DetectorWrapper:
    def __init__(self, max_distance_from_center_x=CAR_DETECTION_MAX_DISTANCE_FROM_CENTER_X,
                 max_distance_from_center_y=CAR_DETECTION_MAX_DISTANCE_FROM_CENTER_Y,
                 detector=None, already_running=False):
        self.already_running = already_running
        self.detector = (detector or Detector())
        self.cached_detected_object = None
        self.prev_seen_class = "no one"
        self.max_distance_from_center_x = max_distance_from_center_x
        self.max_distance_from_center_y = max_distance_from_center_y

    def update_detection(self):
        self.cached_detected_object = self.detector.get_seen_object(cached=self.already_running)
        if self.cached_detected_object is None:
            if self.prev_seen_class is not None:
                logger.info("I'll find ya!")
                self.prev_seen_class = None
        elif self.prev_seen_class is None or self.prev_seen_class != self.cached_detected_object['object_class']:
            logger.info('See ya! (a %s)', self.cached_detected_object['object_class'])
            self.prev_seen_class = self.cached_detected_object['object_class']

    def get_detection_segment(self):
        center_x, center_y = self.cached_detected_object['center']

        if center_x < 0.5 and 0.5 - center_x > self.max_distance_from_center_x:
            result_x = -1
        elif center_x > 0.5 and center_x - 0.5 > self.max_distance_from_center_x:
            result_x = 1
        else:
            result_x = 0

        if center_y < 0.5 and 0.5 - center_y > self.max_distance_from_center_y:
            result_y = -1
        elif center_y > 0.5 and center_y - 0.5 > self.max_distance_from_center_y:
            result_y = 1
        else:
            result_y = 0

        return result_x, result_y

    def get_track_frame(self):
        return {
            'detected_object': self.cached_detected_object,
            'prev_seen_class': self.prev_seen_class,
        }
