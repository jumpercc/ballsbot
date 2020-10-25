import numpy as np
from math import pi

from ballsbot.cloud_to_lines import distance
from ballsbot.lidar import apply_transformation_to_cloud
from ballsbot.cloud_to_lines import cloud_to_lines
from ballsbot.odometry_fix import get_coords_diff
from ballsbot.ndt import NDT

GRID_STEP = 1.  # meter
POINT_RADIUS = GRID_STEP / 2


def get_kp(x, y):
    kp_x = int(round(x / GRID_STEP)) * GRID_STEP
    kp_y = int(round(y / GRID_STEP)) * GRID_STEP
    inside_kp = distance([kp_x, kp_y], [x, y]) <= POINT_RADIUS
    return (kp_x, kp_y), inside_kp


class KeyPoints:
    def __init__(self):
        self.key_points = {}
        self.inside_kp = False
        self.current_kp_id = None
        self.ndt = NDT(grid_size=6., box_size=2., iterations_count=10, optim_step=(0.1, 0.1, 0.01), eps=0.01)

    def _get_nearby_kp_list(self, kp_id):
        result = []
        x, y = kp_id
        for i in [x - 1, x, x + 1]:
            for j in [y - 1, y, y + 1]:
                cur_kp_id = (i, j)
                if cur_kp_id in self.key_points:
                    result.append(cur_kp_id)
        return result

    def _get_pose_fix_lines(self, lines, nearby_kp_list, lidar_points, transformation):
        transformation = np.array(transformation)
        results = []
        for kp_id in nearby_kp_list:
            key_point = self.key_points[kp_id]

            shift = np.array([key_point['pose'][0], key_point['pose'][1]])
            result_diff = None
            for _ in range(10):
                diff = get_coords_diff(key_point['lines'], lines, shift)
                if diff is None:
                    break

                if result_diff is None:
                    result_diff = np.array([0., 0., 0.])
                result_diff += np.array(diff)
                shift += result_diff[:2]
                new_lines = cloud_to_lines(
                    apply_transformation_to_cloud(
                        lidar_points, np.array(transformation) + result_diff
                    )
                )
                if len(new_lines) < 3:
                    break
                else:
                    lines = new_lines

                if abs(diff[0]) < 0.01 and abs(diff[1]) < 0.01 and abs(diff[2]) < 0.01:
                    break

            if result_diff is not None:
                results.append(result_diff)
        if len(results) == 0:
            return None
        return np.median(np.array(results), axis=0)

    def get_fixed_pose_lines(self, pose, lidar_points):
        kp_id, inside_kp = get_kp(pose['x'], pose['y'])
        if not inside_kp:
            self.inside_kp = False
            self.current_kp_id = None
            return

        if self.inside_kp and self.current_kp_id == kp_id:
            return

        lines = cloud_to_lines(
            apply_transformation_to_cloud(
                lidar_points, [pose['x'], pose['y'], pose['teta']]
            )
        )
        if len(lines) < 3:
            return

        has_fixes = False
        fixed_pose = np.array([pose['x'], pose['y'], pose['teta']])
        nearby_kp_list = self._get_nearby_kp_list(kp_id)
        if len(nearby_kp_list) > 0:
            pose_fix = self._get_pose_fix_lines(
                lines, nearby_kp_list, lidar_points,
                [pose['x'], pose['y'], pose['teta']]
            )
            if pose_fix is not None:
                has_fixes = True
                fixed_pose += pose_fix
                kp_id, _ = get_kp(fixed_pose[0], fixed_pose[1])

        if kp_id not in self.key_points:
            if has_fixes:
                lines = cloud_to_lines(
                    apply_transformation_to_cloud(
                        lidar_points, fixed_pose
                    )
                )
            self.key_points[kp_id] = {
                'pose': fixed_pose,
                'lines': lines,
            }

        self.inside_kp = inside_kp
        self.current_kp_id = kp_id

        return {
            'x': fixed_pose[0],
            'y': fixed_pose[1],
            'teta': fixed_pose[2],
        }

    def _get_pose_fix_ndt(self, raw_pose, lidar_points, nearby_kp_list):
        results = []
        for kp_id in nearby_kp_list:
            key_point = self.key_points[kp_id]
            pose_diff = raw_pose - key_point['pose']

            dx, dy, dteta, converged, score = self.ndt.get_optimal_transformation(
                key_point['points'],
                lidar_points,
                start_point=pose_diff
            )
            if converged != 1.:
                continue
            elif score > 0.5:
                continue
            elif abs(dx) > 2. or abs(dy) > 2. or abs(dteta) > pi / 4:
                continue

            print('{}: {}'.format(kp_id, np.array([dx, dy, dteta]) - pose_diff))
            results.append(
                np.array([dx, dy, dteta]) - pose_diff
            )
        if len(results) == 0:
            return None
        return np.median(np.array(results), axis=0)

    def get_fixed_pose_ndt(self, pose, lidar_points):
        kp_id, inside_kp = get_kp(pose['x'], pose['y'])
        if not inside_kp:
            self.inside_kp = False
            self.current_kp_id = None
            return

        if self.inside_kp and self.current_kp_id == kp_id:
            return

        fixed_pose = np.array([pose['x'], pose['y'], pose['teta']])
        nearby_kp_list = self._get_nearby_kp_list(kp_id)
        if len(nearby_kp_list) > 0:
            pose_fix = self._get_pose_fix_ndt(
                fixed_pose,
                apply_transformation_to_cloud(
                    lidar_points, -fixed_pose
                ),
                nearby_kp_list
            )
            if pose_fix is not None:
                fixed_pose += pose_fix
                kp_id, _ = get_kp(fixed_pose[0], fixed_pose[1])

        if kp_id not in self.key_points:
            self.key_points[kp_id] = {
                'pose': fixed_pose,
                'points': apply_transformation_to_cloud(
                    lidar_points, -fixed_pose
                ),
            }

        self.inside_kp = inside_kp
        self.current_kp_id = kp_id

        return {
            'x': fixed_pose[0],
            'y': fixed_pose[1],
            'teta': fixed_pose[2],
        }
