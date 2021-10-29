from ballsbot.ros_messages import get_ros_messages


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

    def get_seen_object(self):
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
