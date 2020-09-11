import json
import ballsbot.drawing as drawing
from ballsbot.lidar import Lidar
from ballsbot.utils import keep_rps
from ballsbot.cloud_to_lines import cloud_to_lines
import random
from copy import deepcopy

from ballsbot.keypoints import KeyPoints


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
            tail_lines = all_lines
        all_lines += lines

        drawing.update_image_abs_coords(
            image, poses[0:i], points, self_position, only_nearby_meters, figsize=(12, 10),
            lines=lines, tail_lines=tail_lines,
        )


def fix_and_rerun_track_lines(
        image_raw, image_fixed,
        file_path='/home/jumper/projects/ballsbot/poses.json',
        only_nearby_meters=10
):
    with open(file_path, 'r') as hf:
        poses = json.loads(hf.read())
    original_poses = deepcopy(poses)

    lidar = Lidar()
    self_position = lidar.calibration_to_xywh(lidar.calibration)
    key_points = KeyPoints()

    all_points = []
    original_all_points = []
    pose_fix = None
    ts = None
    for i in range(0, len(poses) - 1):
        pose = poses[i]
        original_pose = original_poses[i]
        if 'points' not in pose:
            continue

        ts = keep_rps(ts, fps=5)

        tail_weight = 3
        if len(all_points) > len(pose['points']) * tail_weight:
            tail_points = random.sample(all_points, len(pose['points']) * tail_weight)
            original_tail_points = random.sample(original_all_points, len(original_pose['points']) * tail_weight)
        else:
            tail_points = []
            original_tail_points = []

        raw_points = lidar.apply_transformation_to_cloud(
            original_pose['points'],
            [original_pose['x'], original_pose['y'], original_pose['teta']]
        )
        drawing.update_image_abs_coords(
            image_raw, original_poses[0:i], raw_points, self_position, only_nearby_meters, figsize=(12, 10),
            tail_points=original_tail_points
        )
        original_all_points += raw_points

        fixed_pose = key_points.get_fixed_pose_lines(pose, pose['points'])
        if fixed_pose is not None:
            pose_fix = {
                'x': fixed_pose['x'] - pose['x'],
                'y': fixed_pose['y'] - pose['y'],
                'teta': fixed_pose['teta'] - pose['teta'],
            }
            pose['x'] = fixed_pose['x']
            pose['y'] = fixed_pose['y']
            pose['teta'] = fixed_pose['teta']
        elif pose_fix is not None:
            pose['x'] += pose_fix['x']
            pose['y'] += pose_fix['y']
            pose['teta'] += pose_fix['teta']

        points = lidar.apply_transformation_to_cloud(
            pose['points'],
            [pose['x'], pose['y'], pose['teta']]
        )
        all_points += points

        drawing.update_image_abs_coords(
            image_fixed, poses[0:i], points, self_position, only_nearby_meters, figsize=(12, 10),
            tail_points=tail_points,
            lines=cloud_to_lines(
                key_points.lidar.apply_transformation_to_cloud(
                    pose['points'], [pose['x'], pose['y'], pose['teta']]
                )
            )
        )


def fix_and_rerun_track_ndt(
        image_raw, image_fixed,
        file_path='/home/jumper/projects/ballsbot/poses.json',
        only_nearby_meters=10
):
    with open(file_path, 'r') as hf:
        poses = json.loads(hf.read())
    original_poses = deepcopy(poses)

    lidar = Lidar()
    self_position = lidar.calibration_to_xywh(lidar.calibration)
    key_points = KeyPoints()

    all_points = []
    original_all_points = []
    pose_fix = None
    ts = None
    for i in range(0, len(poses) - 1):
        pose = poses[i]
        original_pose = original_poses[i]
        if 'points' not in pose:
            continue

        ts = keep_rps(ts, fps=5)

        tail_weight = 3
        if len(all_points) > len(pose['points']) * tail_weight:
            tail_points = random.sample(all_points, len(pose['points']) * tail_weight)
            original_tail_points = random.sample(original_all_points, len(original_pose['points']) * tail_weight)
        else:
            tail_points = []
            original_tail_points = []

        raw_points = lidar.apply_transformation_to_cloud(
            original_pose['points'],
            [original_pose['x'], original_pose['y'], original_pose['teta']]
        )
        drawing.update_image_abs_coords(
            image_raw, original_poses[0:i], raw_points, self_position, only_nearby_meters, figsize=(12, 10),
            tail_points=original_tail_points
        )
        original_all_points += raw_points

        fixed_pose = key_points.get_fixed_pose_ndt(pose, pose['points'])
        if fixed_pose is not None:
            pose_fix = {
                'x': fixed_pose['x'] - pose['x'],
                'y': fixed_pose['y'] - pose['y'],
                'teta': fixed_pose['teta'] - pose['teta'],
            }
            print(pose_fix)
            pose['x'] = fixed_pose['x']
            pose['y'] = fixed_pose['y']
            pose['teta'] = fixed_pose['teta']
        elif pose_fix is not None:
            pose['x'] += pose_fix['x']
            pose['y'] += pose_fix['y']
            pose['teta'] += pose_fix['teta']

        points = lidar.apply_transformation_to_cloud(
            pose['points'],
            [pose['x'], pose['y'], pose['teta']]
        )
        all_points += points

        drawing.update_image_abs_coords(
            image_fixed, poses[0:i], points, self_position, only_nearby_meters, figsize=(12, 10),
            tail_points=tail_points,
        )
