# -*- coding: utf-8 -*-
# distutils: sources = ballsbot/grometry.cpp

from libcpp.vector cimport vector
from libcpp.string cimport string

cdef extern from "ballsbot/cam_detector.h":
    cdef struct DetectionPoint:
        DetectionPoint(double, double)
        double x
        double y

    cdef struct Detection:
        Detection(string, double, DetectionPoint, DetectionPoint)
        string object_class
        double confidence
        DetectionPoint bottom_left
        DetectionPoint top_right

    cdef cppclass _CamDetector "CamDetector":
        _CamDetector()
        void StartUp(string)
        vector[Detection] Detect()

