# distutils: language=c++
# cython: language_level=3

import pathlib
from libcpp.vector cimport vector
from libcpp.string cimport string
from ballsbot_detection cimport _CamDetector, Detection, DetectionPoint

cdef class CamDetector:
    cdef _CamDetector thisobj
    cdef _CamDetector *thisptr

    def __cinit__(self):
        self.thisptr = &self.thisobj

    def __dealloc__(self):
        pass

    def start_up(self):
        cdef string net_files_directory = str(
            pathlib.Path(__file__).parent.absolute()
        ).encode('UTF-8')
        self.thisobj.StartUp(net_files_directory)

    def detect(self):
        cdef vector[Detection] cpp_result = self.thisobj.Detect()
        result = []

        for it in cpp_result:
            result.append({
                'object_class': it.object_class.decode('UTF-8'),
                'confidence': it.confidence,
                'bottom_left': (it.bottom_left.x, it.bottom_left.y),
                'top_right': (it.top_right.x, it.top_right.y),
            })

        return result

detector = None

def startup_detection():
    global detector
    detector = CamDetector()
    detector.start_up()

def detect():
    global detector
    return detector.detect()

