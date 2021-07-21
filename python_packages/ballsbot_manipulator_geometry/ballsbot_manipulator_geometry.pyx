# distutils: language=c++
# cython: language_level=3

from libcpp.vector cimport vector
from pcl cimport PointXYZ

cdef extern from "ballsbot_manipulator_geometry.h":
    cdef vector[PointXYZ] ApplyRotations(vector[PointXYZ] default_points, vector[PointXYZ] rotations)

def apply_rotations_wrapper(
    default_points,
    rotations
):
    cdef size_t n = len(default_points)
    cdef vector[PointXYZ] default_points_cpp = vector[PointXYZ](n)
    for i in range(n):
        default_points_cpp[i].x = default_points[i][0]
        default_points_cpp[i].y = default_points[i][1]
        default_points_cpp[i].z = default_points[i][2]

    n = len(rotations)
    cdef vector[PointXYZ] rotations_cpp = vector[PointXYZ](n)
    for i in range(n):
        rotations_cpp[i].x = rotations[i][0]
        rotations_cpp[i].y = rotations[i][1]
        rotations_cpp[i].z = rotations[i][2]

    cdef vector[PointXYZ] result_cpp = ApplyRotations(default_points_cpp, rotations_cpp)
    return [(it.x, it.y, it.z) for it in result_cpp]

