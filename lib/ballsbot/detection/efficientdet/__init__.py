import json
import numpy as np
import pathlib

from ballsbot.detection.efficientdet.model import efficientdet
from ballsbot.detection.efficientdet.utils import preprocess_image, postprocess_boxes
from ballsbot.detection.efficientdet.utils.draw_boxes import draw_boxes


def read_classes():
    coco_classes = json.load(open(
        str(pathlib.Path(__file__).parent.absolute()) + '/coco_90.json', 'r'
    ))
    return {value['id'] - 1: value['name'] for value in coco_classes.values()}


def get_model_path():
    return str(pathlib.Path(__file__).parent.absolute()) + '/efficientdet-d0.h5'


phi = 0
weighted_bifpn = True
model_path = get_model_path()
image_size = 512
classes = read_classes()
num_classes = 90
colors = [np.random.randint(0, 256, 3).tolist() for _ in range(num_classes)]


class EfficientDet:
    def __init__(self, score_threshold=0.3):
        self.score_threshold = score_threshold

        _, self.model = efficientdet(
            phi=phi,
            weighted_bifpn=weighted_bifpn,
            num_classes=num_classes,
            score_threshold=score_threshold
        )
        self.model.load_weights(model_path, by_name=True)

    def _get_predictions(self, image):
        image = image[:, :, ::-1]  # BGR -> RGB
        h, w = image.shape[:2]
        image, scale = preprocess_image(image, image_size=image_size)

        boxes, scores, labels = self.model.predict_on_batch([np.expand_dims(image, axis=0)])
        boxes, scores, labels = np.squeeze(boxes), np.squeeze(scores), np.squeeze(labels)

        boxes = postprocess_boxes(boxes=boxes, scale=scale, height=h, width=w)
        indices = np.where(scores[:] > self.score_threshold)[0]
        boxes = boxes[indices]
        labels = labels[indices]

        return boxes, indices, labels, scores

    def draw_predictions(self, image):
        src_image = image.copy()
        boxes, indices, labels, scores = self._get_predictions(image)
        draw_boxes(src_image, boxes, scores, labels, colors, classes)
        return src_image
