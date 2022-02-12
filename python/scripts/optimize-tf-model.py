import argparse
import numpy as np
from tensorflow.python.compiler.tensorrt import trt_convert as trt


def my_input_fn():
    yield np.random.normal(size=(1, 320, 320, 3)).astype(np.uint8),


def main():
    parser = argparse.ArgumentParser(description='Convert TensorFlow model to optimized  version via TensorRT.')
    parser.add_argument(
        '--input-model-directory',
        dest='input_saved_model_dir',
        help='input saved model directory path',
        required=True,
    )
    parser.add_argument(
        '--output-model-directory',
        dest='output_saved_model_dir',
        help='output saved model directory path',
        required=True,
    )
    args = parser.parse_args()

    conversion_params = trt.DEFAULT_TRT_CONVERSION_PARAMS
    conversion_params = conversion_params._replace(
        max_workspace_size_bytes=(1 << 30)
    )
    conversion_params = conversion_params._replace(precision_mode="FP16")

    converter = trt.TrtGraphConverterV2(
        input_saved_model_dir=args.input_saved_model_dir,
        conversion_params=conversion_params
    )
    converter.convert()

    converter.build(input_fn=my_input_fn)
    converter.save(args.output_saved_model_dir)


if __name__ == '__main__':
    main()
