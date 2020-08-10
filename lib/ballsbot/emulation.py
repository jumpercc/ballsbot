import json
import ballsbot.drawing as drawing
from ballsbot.lidar import Lidar
from ballsbot.ndt import NDT
from ballsbot.utils import keep_rps
import random


def rerun_track(image, file_path='/home/jumper/projects/ballsbot/poses.json', only_nearby_meters=10):
    with open(file_path, 'r') as hf:
        poses = json.loads(hf.read())

    lidar = Lidar()
    self_position = lidar.calibration_to_xywh(lidar.calibration)
    ndt = NDT(grid_size=8., box_size=1., iterations_count=20, optim_step=(0.05, 0.05, 0.01), eps=0.01)

    all_points = []
    ts = None
    for i in range(0, len(poses) - 1):
        pose = poses[i]
        if 'points' not in pose:
            continue

        ts = keep_rps(ts, fps=5)

        points = ndt.apply_transformation_to_cloud(
            pose['points'],
            [pose['x'], pose['y'], pose['teta']]
        )

        tail_weight = 3
        if len(all_points) > len(points) * tail_weight:
            tail_points = random.sample(all_points, len(points) * tail_weight)
        else:
            tail_points = []
        all_points += points

        drawing.update_image_abs_coords(
            image, poses[0:i], points, self_position, only_nearby_meters, figsize=(12, 10), tail_points=tail_points
        )
