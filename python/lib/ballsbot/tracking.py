from collections import deque
from ballsbot_localization import ballsbot_localization as grid
from ballsbot.utils import keep_rps, run_as_thread
from ballsbot.lidar import calibration_to_xywh
from ballsbot.config import TURN_DIAMETER, FROM_LIDAR_TO_CENTER, CAR_WIDTH, CAR_LENGTH, ENGINE_NEED_MANUAL_BREAKING, \
    FROM_LIDAR_TO_PIVOT_CENTER
from ballsbot.ros_messages import get_ros_messages


class Tracker:
    def __init__(self, lidar):
        self.fps = 4
        self.current_pose = None
        self.lidar = lidar
        self.lidar_frames_processed = 0
        self.position = calibration_to_xywh(lidar.get_calibration())
        self.lidar_deque = deque(maxlen=self.fps * 3)  # for 3 seconds
        self.lidar_deque_empty_times = 0
        self.pose_deque = deque(maxlen=self.fps * 3)  # for 3 seconds
        self.pose_deque_empty_times = 0
        self.sync_eps = 0.25  # seconds
        self.running = False
        self.messenger = get_ros_messages()

    def stop(self):
        self.running = False

    def start(self):
        self.running = True
        self.lidar.start()
        run_as_thread(self._start_tracking)

    def _update_pose(self):
        data = self.messenger.get_message_data('pose')
        if data:
            self.current_pose = {
                'imu_ts': data.imu_ts.to_sec(),
                'odometry_ts': data.odometry_ts.to_sec(),
                'self_ts': data.header.stamp.to_sec(),
                'x': data.x,
                'y': data.y,
                'teta': data.teta,
            }
            self.current_pose['ts'] = self.current_pose['imu_ts']
            self.pose_deque.appendleft(self.current_pose)

    def get_current_pose(self):
        return self.current_pose

    def _update_lidar_points(self):
        pose = self.get_current_pose()
        if pose:
            self.lidar_deque.appendleft(self.lidar.tick_get_points())

    def _start_tracking(self):
        ts = None
        while True:
            ts = keep_rps(ts, fps=self.fps)
            if not self.running:
                break
            self._update_pose()
            self._update_lidar_points()
            if self.lidar_deque and self.pose_deque:  # let's find first close enough pose and points
                lidar_it = self.lidar_deque.pop()
                pose_it = self.pose_deque.pop()
                while True:
                    if abs(lidar_it['ts'] - pose_it['ts']) <= self.sync_eps:
                        self.lidar.tick_update_grid(pose_it, lidar_it['points'], lidar_it['ts'])
                        if pose_it != self.current_pose:
                            self.lidar.tick_update_grid(self.current_pose, None, self.current_pose['ts'])
                        self.lidar_frames_processed += 1
                        pose_it = None
                        lidar_it = None
                        break
                    elif lidar_it['ts'] > pose_it['ts']:
                        if self.pose_deque:
                            pose_it = self.pose_deque.pop()
                        else:
                            pose_it = None
                            break
                    else:
                        if self.lidar_deque:
                            lidar_it = self.lidar_deque.pop()
                        else:
                            lidar_it = None
                            break
                if lidar_it:  # return unused
                    self.lidar_deque.append(lidar_it)
                    self.pose_deque_empty_times += 1
                elif pose_it:
                    self.pose_deque.append(pose_it)
                    self.lidar_deque_empty_times += 1

    def get_picture_params(self, with_free_tiles=False):
        if self.lidar_frames_processed:
            points = self.lidar.get_lidar_points(cached=True, absolute_coords=True)
            poses = grid.get_poses()
            poses.append(self.get_current_pose())
            if with_free_tiles:
                grid.get_directions_weights(  # no result required
                    self.lidar.get_points_ts(),
                    get_car_info(),
                )
                free_tile_centers = grid.debug_get_free_tile_centers()
                target_point = grid.debug_get_target_point()
                return poses, points, self.position, free_tile_centers, target_point
            else:
                return poses, points, self.position, None, None
        else:
            return ()

    def get_track_frame(self):
        return {
            'current_pose': self.current_pose,
            'lidar_frames_processed': self.lidar_frames_processed,
            'lidar_deque_size': len(self.lidar_deque),
            'lidar_deque_empty_times': self.lidar_deque_empty_times,
            'pose_deque_size': len(self.pose_deque),
            'pose_deque_empty_times': self.pose_deque_empty_times,
        }


def get_car_info():
    return {
        'to_car_center': FROM_LIDAR_TO_CENTER,
        'turn_radius': TURN_DIAMETER / 2.,
        'to_pivot_center': FROM_LIDAR_TO_PIVOT_CENTER,
        'car_width': CAR_WIDTH,
        'car_length': CAR_LENGTH,
        'engine_need_manual_breaking': ENGINE_NEED_MANUAL_BREAKING,
    }
