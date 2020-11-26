# distutils: language=c++
# cython: language_level=3

import pathlib
from libcpp.vector cimport vector
from libcpp.string cimport string
from ballsbot_cpp cimport Point, Distance, Direction, PointCloud, DirectionsWeights, \
    Pose, CarInfo, GridKey, _Grid, FreeDistances

def distance(p1_raw, p2_raw):
    cdef Point p1
    p1.x = p1_raw[0]
    p1.y = p1_raw[1]

    cdef Point p2
    p2.x = p2_raw[0]
    p2.y = p2_raw[1]

    return Distance(p1, p2)

cdef class Grid:
    cdef _Grid thisobj
    cdef _Grid *thisptr

    def __cinit__(self):
        self.thisptr = &self.thisobj

    def __dealloc__(self):
        pass

    def update_grid(self, a_cloud, a_pose):
        cdef size_t n = len(a_cloud)
        cdef PointCloud cpp_cloud = PointCloud(n)
        for i in range(n):
            cpp_cloud.at(i).x = a_cloud[i][0]
            cpp_cloud.at(i).y = a_cloud[i][1]

        cdef Pose cpp_pose
        cpp_pose.x = a_pose['x']
        cpp_pose.y = a_pose['y']
        cpp_pose.teta = a_pose['teta']

        self.thisobj.UpdateGrid(cpp_cloud, cpp_pose)

    def get_directions_weights(self, a_pose, car_info, free_distances):
        cdef CarInfo cpp_car_info
        cpp_car_info.to_car_center = car_info['to_car_center']
        cpp_car_info.turn_radius = car_info['turn_radius']

        cdef Pose cpp_pose
        cpp_pose.x = a_pose['x']
        cpp_pose.y = a_pose['y']
        cpp_pose.teta = a_pose['teta']

        cdef FreeDistances cpp_free_distances = FreeDistances(len(free_distances))
        for i, pair in enumerate(free_distances.items()):
            cpp_free_distances[i].first.steering = pair[0][0]
            cpp_free_distances[i].first.throttle = pair[0][1]
            cpp_free_distances[i].second = pair[1]

        cdef DirectionsWeights cpp_result = self.thisobj.GetDirectionsWeights(
            cpp_pose, cpp_car_info, cpp_free_distances)
        result = {}
        for it in cpp_result:
            key = (it.first.steering, it.first.throttle)
            result[key] = it.second
        return result

    def get_sectors_map(self, a_pose, car_info, half_size):
        cdef CarInfo cpp_car_info
        cpp_car_info.to_car_center = car_info['to_car_center']
        cpp_car_info.turn_radius = car_info['turn_radius']

        cdef Pose cpp_pose
        cpp_pose.x = a_pose['x']
        cpp_pose.y = a_pose['y']
        cpp_pose.teta = a_pose['teta']

        cdef double cpp_half_size = half_size

        cdef vector[vector[size_t]] cpp_result = self.thisobj.GetSectorsMap(
            cpp_pose, cpp_car_info, cpp_half_size)
        result = [[y for y in x] for x in cpp_result]

        return result

    def get_cell_weight(self, grid_key):
        cdef GridKey cpp_grid_key
        cpp_grid_key.x = grid_key[0]
        cpp_grid_key.y = grid_key[1]
        return self.thisobj.GetCellWeight(cpp_grid_key)

grid = None

def update_grid(a_cloud, a_pose):
    global grid
    if grid is None:
        grid = Grid()
    grid.update_grid(a_cloud, a_pose)

def get_directions_weights(a_pose, car_info, free_distances):
    global grid
    if grid is None:
        grid = Grid()
    return grid.get_directions_weights(a_pose, car_info, free_distances)

def get_cell_weight(grid_key):
    global grid
    if grid is None:
        grid = Grid()
    return grid.get_cell_weight(grid_key)

def get_sectors_map(a_pose, car_info, half_size):
    global grid
    if grid is None:
        grid = Grid()
    return grid.get_sectors_map(a_pose, car_info, half_size)

def reset_grid():
    global grid
    grid = Grid()

