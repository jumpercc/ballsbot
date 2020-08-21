from python_pcl_ndt import python_pcl_ndt


class NDT:
    def __init__(self, grid_size=8., box_size=1., iterations_count=20, optim_step=(0.2, 0.2, 0.01), eps=0.01):
        self.grid_size = grid_size
        self.box_size = box_size
        self.iterations_count = iterations_count
        self.optim_step = optim_step
        self.eps = eps

    def get_optimal_transformation(self, cloud_one, cloud_two, start_point=(0., 0., 0.)):
        return python_pcl_ndt.get_transformation_vector_wrapper(
            cloud_one,
            cloud_two,
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
