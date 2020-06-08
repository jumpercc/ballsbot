import numpy as np
from math import floor
from math import cos, sin, pi, exp, atan2
from scipy.optimize import minimize
from scipy.linalg import eigvals, eig


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

    def get_cell_indexes(self, x, y, shift_x=0., shift_y=0.):
        if x < -self.grid_size + shift_x:
            x_index = 0
        elif x > self.grid_size + shift_x:
            x_index = 2 * self.cells_count - 1
        else:
            x_index = self.cells_count + floor((x - shift_x) / self.box_size)

        if y < -self.grid_size + shift_y:
            y_index = 0
        elif y > self.grid_size + shift_y:
            y_index = 2 * self.cells_count - 1
        else:
            y_index = self.cells_count + floor((y - shift_y) / self.box_size)

        return x_index, y_index

    def get_all_cell_indexes(self, x, y):
        return [
            self.get_cell_indexes(x, y, shift_x=0., shift_y=0.),
            self.get_cell_indexes(x, y, shift_x=self.box_size, shift_y=0.),
            self.get_cell_indexes(x, y, shift_x=0., shift_y=self.box_size),
            self.get_cell_indexes(x, y, shift_x=self.box_size, shift_y=self.box_size),
        ]

    def _fix_covariation(self, covar):
        # To prevent this effect, we check, whether
        # the smaller eigenvalue of Σ is at least 0.001 times the
        # larger eigenvalue. If not, it is set to this value.
        min_covar_eigvalue_mult = 0.001
        eigenvalues, eigtnvectors = eig(covar)
        if eigenvalues[0] < min_covar_eigvalue_mult * eigenvalues[1]:
            diag_eig_values = np.array([
                [eigenvalues[0].real, 0],
                [0, eigenvalues[1].real]
            ])
            eig_bectors_m = np.array(eigtnvectors)
            # set minimum smallest eigenvalue:
            diag_eig_values[0, 0] = diag_eig_values[1, 1] * min_covar_eigvalue_mult
            covar = eig_bectors_m @ diag_eig_values @ eig_bectors_m.T
        return covar

    def cell_cloud(self, a_cloud):
        points = [{}, {}, {}, {}]
        x_points, y_points = a_cloud
        for i in range(len(x_points)):
            x = x_points[i]
            y = y_points[i]
            for j, key in enumerate(self.get_all_cell_indexes(x, y)):
                if key not in points[j]:
                    points[j][key] = [[], []]
                points[j][key][0].append(x)
                points[j][key][1].append(y)

        results = []
        for plist in points:
            mean_m = np.zeros((2 * self.cells_count, 2 * self.cells_count, 2))
            cov_m = np.zeros((2 * self.cells_count, 2 * self.cells_count, 2, 2))
            for key, cell_points in plist.items():
                if len(cell_points[0]) < 3:
                    continue
                i, j = key
                mean_m[i, j] = np.mean(cell_points, axis=1)
                cov_m[i, j] = self._fix_covariation(np.cov(cell_points))

            results.append({'mean': mean_m, 'cov': cov_m})
        return results

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
                x, y = self.apply_transformation(x, y, tr, cos_fi, sin_fi)  # TODO inline for optimization
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

    def _get_score_and_gradients(self, tr, a_cloud, grids):
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
                for i, loc in enumerate(self.get_all_cell_indexes(*a_point)):
                    x_index, y_index = loc
                    mean_v = grids[i]['mean'][x_index, y_index]
                    inv_cov_v = grids[i]['cov'][x_index, y_index]
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
                                    + (-mean_jacobi_m[j].T @ inv_cov_v @ mean_jacobi_m[i])
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
            self.steps.append([tr[0], tr[1], tr[2], score, grad])

        return self.score_cache[tr_key], self.grad_cache[tr_key], self.grad2_cache[tr_key]

    def score_transformation(self, *args):
        return self._get_score_and_gradients(*args)[0]

    def get_gradient_vector(self, *args):
        return self._get_score_and_gradients(*args)[1]

    def get_gradient2_matrix(self, *args):
        return self._get_score_and_gradients(*args)[2]

    def get_optimal_transformation(self, cloud_one, cloud_two, start_point=(0., 0., 0.)):
        self.transformed_cache = {}
        self.score_cache = {}
        self.grad_cache = {}
        self.grad2_cache = {}
        self.steps = []
        grids = self.cell_cloud(cloud_one)
        for it in grids:
            it['cov'] = self.inv_cov(it['cov'])
        return minimize(
            self.score_transformation,
            np.array(start_point),  # TODO better start point from EKF
            args=(cloud_two, grids),
            # method='Newton-CG',
            method='CG',
            jac=self.get_gradient_vector,
            # hess=self.get_gradient2_matrix,
            options={
                'xtol': 0.1,  # FIXME
                'disp': True,
                # 'maxiter': 1,
            }
        ).x

    @staticmethod
    def angle_axis_to_rotation_matrix(m_angle, m_axis):
        sin_axis = sin(m_angle) * m_axis
        c = cos(m_angle)
        cos1_axis = (1. - c) * m_axis

        res = np.zeros((3, 3))

        tmp = cos1_axis[0] * m_axis[1]
        res[0, 1] = tmp - sin_axis[2]
        res[1, 0] = tmp + sin_axis[2]

        tmp = cos1_axis[0] * m_axis[2]
        res[0, 2] = tmp + sin_axis[1]
        res[2, 0] = tmp - sin_axis[1]

        tmp = cos1_axis[1] * m_axis[2]
        res[1, 2] = tmp - sin_axis[0]
        res[2, 1] = tmp + sin_axis[0]

        tmp = cos1_axis * m_axis + c
        for i in range(3):
            res[i, i] = tmp[i]

        return res

    def get_my_optimal_transformation(self, cloud_one, cloud_two, start_point=(0., 0., 0.)):
        self.transformed_cache = {}
        self.score_cache = {}
        self.grad_cache = {}
        self.grad2_cache = {}
        self.steps = []
        grids = self.cell_cloud(cloud_one)
        for it in grids:
            it['cov'] = self.inv_cov(it['cov'])

        init_rotation = self.angle_axis_to_rotation_matrix(start_point[-1], np.array([0., 0., 1.]))
        init_translation = np.array([start_point[0], start_point[1], 0.])
        init_guess = init_translation @ init_rotation

        guess = np.eye(4)  # FIXME
        transformation = guess
        # work with x translation, y translation and z rotation: extending to 3D
        # would be some tricky maths, but not impossible.
        initial_rot = transformation[:3, :3]
        rot_x = initial_rot @ np.array([1., 0., 0.])
        z_rotation = atan2(rot_x[1], rot_x[0])
        xytheta_transformation = np.array([
            transformation[0, 3],
            transformation[1, 3],
            z_rotation
        ])

        newton_lambda = np.array([1., 1., 1.])  # (0.4,0.4,0.1)?
        transformation_epsilon = 0.1
        max_iterations = 35
        nr_iterations = 0
        converged = False
        while not converged:
            previous_transformation = transformation

            score_value, score_grad, score_hessian = self._get_score_and_gradients(
                xytheta_transformation, cloud_two, grids
            )

            delta_transformation = -np.linalg.inv(score_hessian) @ score_grad
            new_transformation = xytheta_transformation + newton_lambda * delta_transformation
            xytheta_transformation = new_transformation

            """
            # update transformation matrix from x, y, theta:
            transformation.block<3,3> (0,0).matrix () = Eigen::Matrix3f (Eigen::AngleAxisf (static_cast<float> (xytheta_transformation[2]), Eigen::Vector3f::UnitZ ()));
            transformation.block<3,1> (0,3).matrix () = Eigen::Vector3f (static_cast<float> (xytheta_transformation[0]), static_cast<float> (xytheta_transformation[1]), 0.0f);
            """

            nr_iterations += 1

            transformation_delta = np.linalg.inv(transformation) * previous_transformation
            """
            double cos_angle = 0.5 * (transformation_delta.coeff (0, 0) + transformation_delta.coeff (1, 1) + transformation_delta.coeff (2, 2) - 1);
            double translation_sqr = transformation_delta.coeff (0, 3) * transformation_delta.coeff (0, 3) +
                                       transformation_delta.coeff (1, 3) * transformation_delta.coeff (1, 3) +
                                       transformation_delta.coeff (2, 3) * transformation_delta.coeff (2, 3);

            if (nr_iterations_ >= max_iterations_ ||
                ((transformation_epsilon_ > 0 && translation_sqr <= transformation_epsilon_) && (transformation_rotation_epsilon_ > 0 && cos_angle >= transformation_rotation_epsilon_)) ||
                ((transformation_epsilon_ <= 0)                                             && (transformation_rotation_epsilon_ > 0 && cos_angle >= transformation_rotation_epsilon_)) ||
                ((transformation_epsilon_ > 0 && translation_sqr <= transformation_epsilon_) && (transformation_rotation_epsilon_ <= 0)))
              converged_ = true;
            final_transformation_ = transformation;
            """
