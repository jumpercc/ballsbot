# -*- coding: utf-8 -*-
# distutils: sources = lib/grometry.cpp

from libcpp.vector cimport vector
from libcpp.unordered_map cimport unordered_map
from libcpp cimport bool

cdef extern from "src/common.h":
    cdef struct Point:
        Point(double, double)
        double x
        double y

cdef extern from "src/grid.h":
    cdef struct Direction:
        Direction(double, double)
        double steering
        double throttle

ctypedef vector[Point] PointCloud
ctypedef unordered_map[Direction, double] DirectionsWeights

cdef extern from "src/grid.h":
    cdef struct Pose:
        Pose(double, double, double)
        double x
        double y
        double teta

    cdef struct CarInfo:
        CarInfo(double, double, double)
        double to_car_center
        double to_pivot_center
        double turn_radius
        double car_width
        double car_length

    cdef struct Voxel:
        Voxel(bool, double)
        bool occupied
        double last_seen_occupied_ts

    cdef struct TileKey:
        TileKey(int, int)
        int x
        int y

    cdef cppclass Tile:
        Tile()
        vector[vector[Voxel]] DebugGetVoxels()
        double GetVisitedTs()

    cdef cppclass _Grid "Grid":
        _Grid()
        void UpdateGrid(PointCloud, Pose, double)
        void UpdatePose(Pose)
        unordered_map[TileKey, Tile] DebugGetTiles()
        PointCloud GetSparsePointCloud(double, double, bool)
        DirectionsWeights GetDirectionsWeights(double, CarInfo)
        void CleanUpGrid(double)
        vector[Pose] GetPoses()
        PointCloud DebugGetFreeTileCenters(bool)
        Point DebugGetTargetPoint(bool)

cdef extern from "src/free_distances.h":
    cdef DirectionsWeights DebugGetFreeDistances(_Grid, double, CarInfo)

