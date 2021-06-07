# -*- coding: utf-8 -*-

# Eigen
cdef extern from "Eigen/Eigen" namespace "Eigen" nogil:
    cdef cppclass Matrix4f:
        Matrix4f() except +
        float *data()
        float& element "operator()"(int row, int col)
    cdef cppclass Matrix3f:
        Matrix3f() except +
        float coeff(int row, int col)
        float *data()
        float& element "operator()"(int row, int col)
    cdef cppclass Vector4f:
        Vector4f() except +
        Vector4f(float c0, float c1, float c2, float c3) except +
        float *data()
        float& element "operator()"(int row, int col)
    cdef cppclass Vector3f:
        Vector3f() except +
        Vector3f(float c0, float c1, float c2) except +
        float *data()
        float& element "operator()"(int row, int col)
    cdef cppclass Vector3i:
        Vector3i() except +
        int *data()
        int& element "operator()"(int row, int col)
    cdef cppclass Vector3d:
        Vector3d() except +
        Vector3d(double c0, double c1, double c2) except +
        double coeff(int row, int col)
        double& element "operator()"(int row, int col)
        double *data()
    cdef cppclass Quaternionf:
        Quaternionf()
        Quaternionf(float, float, float, float)
        float w()
        float x()
        float y()
        float z()
    cdef cppclass Affine3f:
        Affine3f() except +
        float *data()
    cdef cppclass aligned_allocator[T]:
        pass