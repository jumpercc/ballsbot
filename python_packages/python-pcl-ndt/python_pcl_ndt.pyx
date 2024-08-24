# distutils: language=c++
# cython: language_level=3

from libcpp cimport bool
from libcpp.vector cimport vector
from python_pcl_ndt cimport PointCloud_t, _Tracker


cdef class PyTracker:
    cdef _Tracker thisobj
    cdef _Tracker *thisptr

    def __cinit__(self):
        self.thisptr = &self.thisobj

    def __dealloc__(self):
        pass

    def set_fast(self, fast):
        cdef bool cpp_fast = fast
        self.thisobj.set_fast(cpp_fast)

    def align_clouds(self, pose, cloud):
        cdef vector[double] cpp_pose = vector[double](3)
        cpp_pose[0] = pose['x']
        cpp_pose[1] = pose['y']
        cpp_pose[2] = pose['teta']

        cdef size_t n = len(cloud)
        cdef PointCloud_t cpp_cloud = PointCloud_t(n, 1)
        for i in range(n):
            cpp_cloud.at(i).x = cloud[i][0]
            cpp_cloud.at(i).y = cloud[i][1]
            cpp_cloud.at(i).z = 0.

        self.thisobj.set_input(cpp_pose, cpp_cloud.makeShared())
        return {
            'x_error': self.thisobj.get_x_error(),
            'y_error': self.thisobj.get_y_error(),
            'teta_error': self.thisobj.get_teta_error(),
            'x': self.thisobj.get_x(),
            'y': self.thisobj.get_y(),
            'teta': self.thisobj.get_teta(),
        }

tracker = None

def align_clouds(pose, cloud, fast):
    global tracker
    if tracker is None:
        tracker = PyTracker()
    tracker.set_fast(fast)
    return tracker.align_clouds(pose, cloud)
