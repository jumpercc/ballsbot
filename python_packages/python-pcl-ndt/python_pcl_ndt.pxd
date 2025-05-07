# -*- coding: utf-8 -*-

from libcpp cimport bool
from libcpp.vector cimport vector
from boost_shared_ptr cimport shared_ptr

cdef extern from "pcl/point_types.h" namespace "pcl" nogil:
    cdef struct PointXYZ:
        PointXYZ()
        float x
        float y
        float z

cdef extern from "pcl/point_cloud.h" namespace "pcl" nogil:
    cdef cppclass PointCloud[T]:
        PointCloud() except +
        PointCloud(unsigned int, unsigned int) except +
        unsigned int width
        unsigned int height
        bool is_dense
        void resize(size_t) except +
        size_t size()
        T& at(size_t) except +
        shared_ptr[PointCloud[T]] makeShared()

ctypedef PointCloud[PointXYZ] PointCloud_t
ctypedef shared_ptr[PointCloud[PointXYZ]] ShPtr_PointCloud_t

cdef extern from "ndt_tracker.h" nogil:
    cdef cppclass _Tracker "Tracker":
        _Tracker()
        void set_fast(bool)
        void set_input(vector[double], ShPtr_PointCloud_t)
        vector[double] get_error()
        vector[double] get_pose()
