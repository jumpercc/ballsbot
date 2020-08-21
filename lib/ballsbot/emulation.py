import json
import ballsbot.drawing as drawing
from ballsbot.lidar import Lidar
from ballsbot.utils import keep_rps
from ballsbot.cloud_to_lines import cloud_to_lines
import random
import numpy as np

import torch
from ballsbot.odometry_fix import get_model, clouds_pair_to_tensor


def rerun_track(image, file_path='/home/jumper/projects/ballsbot/poses.json', only_nearby_meters=10):
    with open(file_path, 'r') as hf:
        poses = json.loads(hf.read())

    lidar = Lidar()
    self_position = lidar.calibration_to_xywh(lidar.calibration)

    all_points = []
    ts = None
    for i in range(0, len(poses) - 1):
        pose = poses[i]
        if 'points' not in pose:
            continue

        ts = keep_rps(ts, fps=5)

        points = lidar.apply_transformation_to_cloud(
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


def rerun_track_lines(image, file_path='/home/jumper/projects/ballsbot/poses.json', only_nearby_meters=10):
    with open(file_path, 'r') as hf:
        poses = json.loads(hf.read())

    lidar = Lidar()
    self_position = lidar.calibration_to_xywh(lidar.calibration)

    ts = None
    all_lines = []
    for i in range(1, len(poses) - 1):
        pose = poses[i]
        if 'points' not in pose:
            continue

        ts = keep_rps(ts, fps=5)

        points = lidar.apply_transformation_to_cloud(
            pose['points'],
            [pose['x'], pose['y'], pose['teta']]
        )

        lines = cloud_to_lines(points)
        tail_size = 100
        if len(all_lines) > tail_size:
            tail_lines = random.sample(all_lines, tail_size)
        else:
            tail_lines = []
        all_lines += lines

        drawing.update_image_abs_coords(
            image, poses[0:i], points, self_position, only_nearby_meters, figsize=(12, 10),
            lines=lines, tail_lines=tail_lines,
        )


def fix_and_rerun_track(image_raw, image_fixed, file_path='/home/jumper/projects/ballsbot/poses.json',
                        only_nearby_meters=10):
    with open(file_path, 'r') as hf:
        poses = json.loads(hf.read())

    lidar = Lidar()
    self_position = lidar.calibration_to_xywh(lidar.calibration)

    device = torch.device('cuda')
    model = get_model().to(device)

    prev_points = []
    all_points = []
    pose_err = None
    ts = None
    for i in range(0, len(poses) - 1):
        pose = poses[i]
        if 'points' not in pose:
            continue

        ts = keep_rps(ts, fps=5)

        tail_weight = 3
        if len(all_points) > len(pose['points']) * tail_weight:
            tail_points = random.sample(all_points, len(points) * tail_weight)
        else:
            tail_points = []

        raw_points = lidar.apply_transformation_to_cloud(
            pose['points'],
            [pose['x'], pose['y'], pose['teta']]
        )
        drawing.update_image_abs_coords(
            image_raw, poses[0:i], raw_points, self_position, only_nearby_meters, figsize=(12, 10),
            tail_points=tail_points
        )

        prev_distance = 20
        if len(prev_points) > 0 and i % prev_distance == 0:
            prev_pose = poses[i - prev_distance]
            tmp_points = lidar.apply_transformation_to_cloud(
                pose['points'],
                np.array([pose['x'], pose['y'], pose['teta']])
                - np.array([prev_pose['x'], prev_pose['y'], prev_pose['teta']])
            )
            pose_err = model(
                clouds_pair_to_tensor(prev_points[1 - prev_distance], tmp_points).to(device)
            ).cpu().data.numpy()[0]
            print(pose_err)
        if pose_err is not None:
            pose['x'] += pose_err[0]
            pose['y'] += pose_err[1]
            pose['teta'] += pose_err[1]
        prev_points.append(pose['points'])

        points = lidar.apply_transformation_to_cloud(
            pose['points'],
            [pose['x'], pose['y'], pose['teta']]
        )
        all_points += points

        drawing.update_image_abs_coords(
            image_fixed, poses[0:i], points, self_position, only_nearby_meters, figsize=(12, 10),
            tail_points=tail_points
        )
