# distutils: language=c++
# cython: language_level=3

import pathlib
from libcpp.vector cimport vector
from libcpp.unordered_map cimport unordered_map
from libcpp cimport bool
from ballsbot_localization cimport Point, PointCloud, Pose, Voxel, TileKey, Tile, _Grid, \
    CarInfo, DirectionsWeights, DebugGetFreeDistances

cdef class Grid:
    cdef _Grid thisobj
    cdef _Grid *thisptr

    def __cinit__(self):
        self.thisptr = &self.thisobj

    def __dealloc__(self):
        pass

    def update_grid(self, a_cloud, a_pose, ts):
        cdef size_t n = len(a_cloud)
        cdef PointCloud cpp_cloud = PointCloud(n)
        for i in range(n):
            cpp_cloud.at(i).x = a_cloud[i][0]
            cpp_cloud.at(i).y = a_cloud[i][1]

        cdef Pose cpp_pose
        cpp_pose.x = a_pose['x']
        cpp_pose.y = a_pose['y']
        cpp_pose.teta = a_pose['teta']

        cdef double cpp_ts = ts

        self.thisobj.UpdateGrid(cpp_cloud, cpp_pose, cpp_ts)

    def update_pose(self, a_pose):
        cdef Pose cpp_pose
        cpp_pose.x = a_pose['x']
        cpp_pose.y = a_pose['y']
        cpp_pose.teta = a_pose['teta']
        self.thisobj.UpdatePose(cpp_pose)

    def debug_get_tiles(self):
        cdef unordered_map[TileKey, Tile] cpp_tiles = self.thisobj.DebugGetTiles()
        cdef vector[vector[Voxel]] cpp_voxels
        result = {}
        for it in cpp_tiles:
            tile_key = (it.first.x, it.first.y)
            cpp_voxels = it.second.DebugGetVoxels()
            voxels = [
                [{
                    'occupied': y.get_filtered_occupation(),
                    'last_seen_occupied': y.last_seen_occupied_ts,
                } for y in x] for x in cpp_voxels
            ]
            result[tile_key] = {
                'last_visited': it.second.GetVisitedTs(),
                'voxels': voxels,
            }
        return result

    def get_sparse_point_cloud(self, current_ts, range_limit, absolute_coords):
        cdef double cpp_ts = current_ts
        cdef double cpp_range_limit = range_limit
        cdef bool cpp_absolute_coords = absolute_coords
        cdef PointCloud cpp_result = self.thisobj.GetSparsePointCloud(
            cpp_ts, cpp_range_limit, cpp_absolute_coords)
        result = []
        for cpp_point in cpp_result:
            result.append((cpp_point.x, cpp_point.y))
        return result
    
    def debug_get_free_tile_centers(self, absolute_coords):
        cdef bool cpp_absolute_coords = absolute_coords
        cdef PointCloud cpp_result = self.thisobj.DebugGetFreeTileCenters(
            cpp_absolute_coords)
        result = []
        for cpp_point in cpp_result:
            result.append([cpp_point.x, cpp_point.y])
        return result
    
    def debug_get_target_point(self, absolute_coords):
        cdef bool cpp_absolute_coords = absolute_coords
        cdef Point cpp_result = self.thisobj.DebugGetTargetPoint(
            cpp_absolute_coords)
        return [cpp_result.x, cpp_result.y]

    def get_directions_weights(self, current_ts, car_info):
        cdef double cpp_ts = current_ts
        cdef CarInfo cpp_car_info
        cpp_car_info.to_car_center = car_info['to_car_center']
        cpp_car_info.to_pivot_center = car_info['to_pivot_center']
        cpp_car_info.turn_radius = car_info['turn_radius']
        cpp_car_info.car_width = car_info['car_width']
        cpp_car_info.car_length = car_info['car_length']

        cdef DirectionsWeights cpp_result = self.thisobj.GetDirectionsWeights(
            cpp_ts, cpp_car_info)
        result = {}
        for it in cpp_result:
            key = (it.first.steering, it.first.throttle)
            result[key] = it.second
        return result

    def get_poses(self):
        cdef vector[Pose] cpp_result = self.thisobj.GetPoses()
        result = []
        for it in cpp_result:
            result.append({
                'x': it.x,
                'y': it.y,
                'teta': it.teta,
            })
        return result

    def debug_get_free_distances(self, current_ts, car_info):
        cdef double cpp_ts = current_ts
        cdef CarInfo cpp_car_info
        cpp_car_info.to_car_center = car_info['to_car_center']
        cpp_car_info.to_pivot_center = car_info['to_pivot_center']
        cpp_car_info.turn_radius = car_info['turn_radius']
        cpp_car_info.car_width = car_info['car_width']
        cpp_car_info.car_length = car_info['car_length']
        cdef _Grid cpp_grid = self.thisobj

        cdef DirectionsWeights cpp_result = DebugGetFreeDistances(
            cpp_grid, cpp_ts, cpp_car_info)
        result = {}
        for it in cpp_result:
            key = (it.first.steering, it.first.throttle)
            result[key] = it.second
        return result

    def clean_up_grid(self, current_ts):
        cdef double cpp_ts = current_ts
        self.thisobj.CleanUpGrid(cpp_ts)

grid = None

def update_grid(a_cloud, a_pose, ts):
    global grid
    if grid is None:
        grid = Grid()
    grid.update_grid(a_cloud, a_pose, ts)

def update_pose(a_pose):
    global grid
    if grid is None:
        grid = Grid()
    grid.update_pose(a_pose)

def debug_get_tiles():
    global grid
    if grid is None:
        grid = Grid()
    return grid.debug_get_tiles()

def get_sparse_point_cloud(current_ts, range_limit, absolute_coords):
    global grid
    if grid is None:
        grid = Grid()
    return grid.get_sparse_point_cloud(current_ts, range_limit, absolute_coords)

def get_directions_weights(current_ts, car_info):
    global grid
    if grid is None:
        grid = Grid()
    return grid.get_directions_weights(current_ts, car_info)

def debug_get_free_distances(current_ts, car_info):
    global grid
    if grid is None:
        grid = Grid()
    return grid.debug_get_free_distances(current_ts, car_info)

def clean_up_grid(current_ts):
    global grid
    if grid is None:
        grid = Grid()
    return grid.clean_up_grid(current_ts)

def get_poses():
    global grid
    if grid is None:
        grid = Grid()
    return grid.get_poses()

def debug_get_free_tile_centers(absolute=True):
    global grid
    if grid is None:
        grid = Grid()
    return grid.debug_get_free_tile_centers(absolute)

def debug_get_target_point(absolute=True):
    global grid
    if grid is None:
        grid = Grid()
    return grid.debug_get_target_point(absolute)

