# -*- coding: utf-8 -*-
# distutils: sources = ballsbot/grometry.cpp

from libcpp.vector cimport vector
from libcpp.unordered_map cimport unordered_map
from libcpp.utility cimport pair
from libcpp.string cimport string

cdef extern from "ballsbot/common.h":
    cdef struct Point:
        Point(double, double)
        double x
        double y

cdef extern from "ballsbot/geometry.h":
    cdef struct LinearCoefficients:
        LinearCoefficients(double, double, double)
        double a
        double b
        double c

cdef extern from "ballsbot/geometry.h":
    cdef double Distance(Point p1, Point p2)

cdef extern from "ballsbot/grid.h":
    cdef struct Direction:
        Direction(double, double)
        double steering
        double throttle

ctypedef vector[Point] PointCloud
ctypedef unordered_map[Direction, double] DirectionsWeights
ctypedef vector[pair[Direction, double]] FreeDistances

cdef extern from "ballsbot/grid.h":
    cdef struct Pose:
        Pose(double, double, double)
        double x
        double y
        double teta

    cdef struct GridKey:
        GridKey(int, int)
        int x
        int y

    cdef struct CarInfo:
        CarInfo(double, double)
        double to_car_center
        double turn_radius

    cdef cppclass _Grid "Grid":
        _Grid()
        void UpdateGrid(PointCloud, Pose)
        DirectionsWeights GetDirectionsWeights(Pose, CarInfo, FreeDistances)
        double GetCellWeight(GridKey)
        vector[vector[size_t]] GetSectorsMap(Pose, CarInfo, double)


cdef extern from "ballsbot/cam_detector.h":
    cdef struct DetectionPoint:
        DetectionPoint(double, double)
        double x
        double y

    cdef struct Detection:
        Detection(string, double, DetectionPoint, DetectionPoint)
        string object_class
        double confidence
        DetectionPoint top_left
        DetectionPoint bottom_right

    cdef cppclass _CamDetector "CamDetector":
        _CamDetector()
        void StartUp(string)
        vector[Detection] Detect()

