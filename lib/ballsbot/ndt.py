import numpy as np
from math import cos, sin
from python_pcl_ndt import python_pcl_ndt


class NDT:
    def __init__(self, grid_size=8., box_size=1., iterations_count=20, optim_step=(0.2, 0.2, 0.01), eps=0.01):
        self.grid_size = grid_size
        self.box_size = box_size
        self.iterations_count = iterations_count
        self.optim_step = optim_step
        self.eps = eps

    @staticmethod
    def apply_transformation(x, y, tr, cos_fi, sin_fi):
        tx, ty, fi = tr
        rotate_m = np.array([
            [cos_fi, -sin_fi],
            [sin_fi, cos_fi]
        ])
        move_m = np.array([[tx], [ty]])
        point = np.array([[x], [y]])
        result = rotate_m @ point + move_m
        return result[:, 0]

    def apply_transformation_to_cloud(self, a_cloud, tr):
        result_x = []
        result_y = []
        x_points, y_points = a_cloud
        cos_fi = cos(tr[-1])
        sin_fi = sin(tr[-1])
        for i in range(len(x_points)):
            x = x_points[i]
            y = y_points[i]
            x, y = self.apply_transformation(x, y, tr, cos_fi, sin_fi)  # TODO inline for optimization
            result_x.append(x)
            result_y.append(y)
        return [result_x, result_y]  # FIXME

    def get_optimal_transformation(self, cloud_one, cloud_two, start_point=(0., 0., 0.)):
        cloud_one_points = list(zip(cloud_one[0], cloud_one[1]))  # FIXME
        cloud_two_points = list(zip(cloud_two[0], cloud_two[1]))  # FIXME

        return python_pcl_ndt.get_transformation_vector_wrapper(
            cloud_one_points,
            cloud_two_points,
            self.iterations_count,  # iter
            self.box_size,  # grid_step
            self.grid_size,  # grid_extent
            self.optim_step[0],  # optim_step_x
            self.optim_step[1],  # optim_step_y
            self.optim_step[2],  # optim_step_theta
            self.eps,  # epsilon
            start_point[0],  # guess_x
            start_point[1],  # guess_y
            start_point[2],  # guess_theta
        )
