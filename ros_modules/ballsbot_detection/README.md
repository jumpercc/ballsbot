# Детектор объектов в камере, модуль для ROS

- берёт данные с камеры (csi, gstreamer)
- распознаёт объекты через SSD MobileNet
- публикует результаты

Ввиду нетривиальности сборки TensorRT кода, собрал его в отдельную библиотеку и положил рядом.
Вот исходники для [libuffssd.so](https://github.com/jumpercc/TensorRT/blob/release/7.1-jetson-nano-crosscompile-ssd/samples/opensource/sampleUffSSD/sampleUffSSD.cpp)
