import numpy as np
from math import floor
from math import cos, sin, pi, exp
from scipy.optimize import minimize
from scipy.linalg import eigvals


class NDT:
    """
    The Normal Distributions Transform
    https://www.researchgate.net/publication/4045903_The_Normal_Distributions_Transform_A_New_Approach_to_Laser_Scan_Matching
    """

    def __init__(self, grid_size=8., box_size=1., shift_fix=0.0001):
        self.grid_size = grid_size
        self.box_size = box_size
        self.shift_fix = shift_fix
        self.cells_count = int(grid_size / box_size)  # half of 1d only
        self.transformed_cache = {}
        self.score_cache = {}
        self.grad_cache = {}
        self.grad2_cache = {}
        self.steps = []

    def get_cell_indexes(self, x, y):
        if x < -self.grid_size:
            x = self.shift_fix - self.grid_size
        elif x > self.grid_size:
            x = self.grid_size - self.shift_fix
        if y < -self.grid_size:
            y = self.shift_fix - self.grid_size
        elif y > self.grid_size:
            y = self.grid_size - self.shift_fix
        return self.cells_count + floor(x / self.box_size), self.cells_count + floor(y / self.box_size)

    def cell_cloud(self, a_cloud):
        points = {}
        x_points, y_points = a_cloud
        for i in range(len(x_points)):
            x = x_points[i]
            y = y_points[i]
            key = self.get_cell_indexes(x, y)
            if key not in points:
                points[key] = [[], []]
            points[key][0].append(x)
            points[key][1].append(y)

        mean_m = np.zeros((2 * self.cells_count, 2 * self.cells_count, 2))
        cov_m = np.zeros((2 * self.cells_count, 2 * self.cells_count, 2, 2))
        for key, cell_points in points.items():
            if len(cell_points[0]) < 3:
                continue
            i, j = key
            mean_m[i, j] = np.mean(cell_points, axis=1)
            cov_m[i, j] = np.cov(cell_points)
        return mean_m, cov_m

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
        tr_key = tuple(tr)
        if tr_key not in self.transformed_cache:
            result_x = []
            result_y = []
            x_points, y_points = a_cloud
            cos_fi = cos(tr[-1])
            sin_fi = sin(tr[-1])
            for i in range(len(x_points)):
                x = x_points[i]
                y = y_points[i]
                x, y = self.apply_transformation(x, y, tr, cos_fi, sin_fi)
                result_x.append(x)
                result_y.append(y)
            self.transformed_cache[tr_key] = [result_x, result_y]
        return self.transformed_cache[tr_key]

    @staticmethod
    def inv_cov(cov_m):
        result = cov_m.copy()
        for i in range(len(cov_m)):
            cov_row = cov_m[i]
            for j in range(len(cov_row)):
                cov_v = cov_row[j]
                if np.count_nonzero(cov_v) > 0:
                    result[i, j] = np.linalg.inv(cov_v)
        return result

    @staticmethod
    def get_jacobi_m(tr):
        tx, ty, fi = tr
        return np.array([
            [1, 0, -tx * sin(fi) - ty * cos(fi)],
            [0, 1, tx * cos(fi) - ty * sin(fi)],
        ]).T

    @staticmethod
    def get_hess_m_item(tr):
        tx, ty, fi = tr
        return np.array([
            -tx * cos(fi) + ty * sin(fi),
            -tx * sin(fi) - ty * cos(fi)
        ])

    def _get_score_and_gradients(self, tr, a_cloud, mean_one, inv_cov_one):
        tr_key = tuple(tr)
        if tr_key not in self.score_cache:
            rotated_cloud_two = self.apply_transformation_to_cloud(a_cloud, tr)
            x_points, y_points = rotated_cloud_two

            mean_jacobi_m = self.get_jacobi_m(tr)
            mean_hess_m_item = self.get_hess_m_item(tr)
            empy_hess_item = np.zeros((2,))

            score = 0.
            grad = np.zeros((3,))
            grad2 = np.zeros((3, 3))

            for point_index in range(len(x_points)):
                a_point = np.array([x_points[point_index], y_points[point_index]])
                x_index, y_index = self.get_cell_indexes(*a_point)
                mean_v = mean_one[x_index, y_index]
                inv_cov_v = inv_cov_one[x_index, y_index]
                if np.count_nonzero(mean_v) == 0:
                    continue

                q = a_point - mean_v
                qt_cvi = q.T @ inv_cov_v
                exp_qt_cvi_q = exp(-0.5 * qt_cvi @ q)
                score -= exp_qt_cvi_q

                for i in range(3):
                    grad[i] += qt_cvi @ mean_jacobi_m[i] * exp_qt_cvi_q

                for i in range(3):
                    for j in range(3):
                        # second derivative only for i == j == 2:
                        hess_item = mean_hess_m_item if (i == j == 2) else empy_hess_item
                        grad2[i, j] += -exp_qt_cvi_q * (
                                (-qt_cvi @ mean_jacobi_m[i]) * (-qt_cvi @ mean_jacobi_m[j])
                                + (-qt_cvi @ hess_item)
                                + (mean_jacobi_m[j].T @ inv_cov_v @ mean_jacobi_m[i])
                        )

            if score != 0.:
                min_eigenvalue = 0.
                eigenvalues = eigvals(grad2)
                for en in eigenvalues:
                    if en.real < min_eigenvalue:
                        min_eigenvalue = en.real
                if min_eigenvalue < 0.:
                    mlambda = 1.1 * min_eigenvalue - 1.
                    grad2 += np.eye(3) * -mlambda
                    eigenvalues = eigvals(grad2)
                assert (eigenvalues[0].real >= 0 and
                        eigenvalues[1].real >= 0 and
                        eigenvalues[2].real >= 0)
            else:
                print("no overlap: try increasing the size or reducing the step of the grid")

            self.score_cache[tr_key] = score
            self.grad_cache[tr_key] = grad
            self.grad2_cache[tr_key] = grad2
            self.steps.append([tr[0], tr[1], score, grad])

        return self.score_cache[tr_key], self.grad_cache[tr_key], self.grad2_cache[tr_key]

    def score_transformation(self, tr, a_cloud, mean_one, inv_cov_one):
        return self._get_score_and_gradients(tr, a_cloud, mean_one, inv_cov_one)[0]

    def get_gradient_vector(self, tr, a_cloud, mean_one, inv_cov_one):
        return self._get_score_and_gradients(tr, a_cloud, mean_one, inv_cov_one)[1]

    def get_gradient2_matrix(self, tr, a_cloud, mean_one, inv_cov_one):
        return self._get_score_and_gradients(tr, a_cloud, mean_one, inv_cov_one)[2]

    def get_optimal_transformation(self, cloud_one, cloud_two, start_point=(0., 0., 0.)):
        self.transformed_cache = {}
        self.score_cache = {}
        self.grad_cache = {}
        self.grad2_cache = {}
        self.steps = []
        mean_one, cov_one = self.cell_cloud(cloud_one)
        inv_cov_one = self.inv_cov(cov_one)
        return minimize(
            self.score_transformation,
            np.array(start_point),  # TODO better start point from EKF
            args=(cloud_two, mean_one, inv_cov_one),
            #bounds=((-1., 1.), (-1., 1.), (-1., 1.)),
            #method='Newton-CG',
            method='CG',
            jac=self.get_gradient_vector,
            #hess=self.get_gradient2_matrix,
            options={
                # 'xtol': 0.05,
                # 'gtol': 0.05,
                'disp': True,
                #'maxiter': 3,
            }
        ).x
