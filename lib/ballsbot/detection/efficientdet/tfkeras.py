from ballsbot.detection.efficientdet.utils import inject_tfkeras_modules, init_tfkeras_custom_objects
from ballsbot.detection.efficientdet.efficientnet import EfficientNetB0 as EfficientNetB0_raw, \
    preprocess_input as preprocess_input_raw

EfficientNetB0 = inject_tfkeras_modules(EfficientNetB0_raw)
EfficientNetB1 = None
EfficientNetB2 = None
EfficientNetB3 = None
EfficientNetB4 = None
EfficientNetB5 = None
EfficientNetB6 = None

preprocess_input = inject_tfkeras_modules(preprocess_input_raw)

init_tfkeras_custom_objects()
