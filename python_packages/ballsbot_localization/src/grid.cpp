#include "grid.h"
#include "point_cloud.h"
#include "geometry.h"
#include "free_distances.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <queue>

const double kVoxelSize = 0.05;  // meters
const double kHalfVoxelSize = kVoxelSize / 2.;
const size_t kVoxelsPerTile = 10;
const double kTileSize = kVoxelsPerTile * kVoxelSize;
const double kHalfTileSize = kTileSize / 2.;
const double kKeepOccupiedInterval = 5.;      // seconds
const double kCleanUpFreeTileInterval = 15.;  // seconds
const double kWeightDonorDistanceMultiplier = 1.2 / 0.44;
const double kUseForWeightsRadius = 4.;  // meters
const int kFreeTilesSectorsCount = 36;
const double kNoTilesButDistanceWeight = 0.01;
const double kTargetTTL = 15.;  // seconds
const double kWideSectorDistance = kVoxelSize / tan(2. * M_PI / kFreeTilesSectorsCount);

Tile::Tile() : voxels_(kVoxelsPerTile) {
    for (size_t i = 0; i < voxels_.size(); ++i) {
        voxels_[i].resize(kVoxelsPerTile);
    }
    visited_ts_ = 0.;
}

void Tile::SetVoxelOccupied(int voxel_x, int voxel_y, double ts) {
    Voxel& voxel = voxels_[voxel_x][voxel_y];
    voxel.occupied = true;
    voxel.last_seen_occupied_ts = ts;
}

void Tile::SetVisited(double ts) {
    visited_ts_ = ts;
}

void Grid::UpdateGrid(PointCloud& cloud, Pose& pose, double ts) {
    poses_.push_back(pose);
    if (poses_.size() > poses_to_keep_) {
        poses_.pop_front();
    }
    CloudTransformation transformation = {pose.x, pose.y, pose.teta};
    auto points = ApplyTransformation(cloud, transformation);

    TileKey tile_key;
    int voxel_x, voxel_y;
    for (auto point : points) {
        tile_key.x = static_cast<int>(std::floor(point.x / kTileSize));
        tile_key.y = static_cast<int>(std::floor(point.y / kTileSize));

        if (tiles_.find(tile_key) == tiles_.end()) {
            tiles_[tile_key] = Tile();
        }
        Tile& tile = tiles_[tile_key];

        voxel_x = static_cast<int>(std::floor((point.x - kTileSize * tile_key.x) / kVoxelSize));
        voxel_y = static_cast<int>(std::floor((point.y - kTileSize * tile_key.y) / kVoxelSize));
        tile.SetVoxelOccupied(voxel_x, voxel_y, ts);
    }

    tile_key.x = static_cast<int>(std::floor(pose.x / kTileSize));
    tile_key.y = static_cast<int>(std::floor(pose.y / kTileSize));
    if (tiles_.find(tile_key) == tiles_.end()) {
        tiles_[tile_key] = Tile();
    }
    tiles_[tile_key].SetVisited(ts);
}

void Grid::UpdatePose(Pose& pose) {
    poses_.push_back(pose);
    if (poses_.size() > poses_to_keep_) {
        poses_.pop_front();
    }
}

Pose Grid::GetCurrentPose() const {
    if (poses_.empty()) {
        throw std::runtime_error("you need to call UpdateGrid first");
    }
    Pose pose = poses_.back();
    return pose;
}

std::vector<Pose> Grid::GetPoses() const {
    std::vector<Pose> result;
    result.reserve(poses_.size());
    for (Pose pose : poses_) {
        result.push_back(pose);
    }
    return result;
}

PointCloud Grid::GetSparsePointCloud(double current_ts, double range_limit,
                                     bool absolute_coords) const {
    auto pose = GetCurrentPose();
    PointCloud result;

    double min_ts = current_ts - kKeepOccupiedInterval;
    TileKey tile_key;
    for (auto tile_pair : tiles_) {
        tile_key = tile_pair.first;
        Tile& tile = tile_pair.second;
        std::vector<std::vector<Voxel>>& voxels = tile.voxels_;
        for (size_t x = 0; x < voxels.size(); ++x) {
            for (size_t y = 0; y < voxels.size(); ++y) {
                Voxel& voxel = voxels[x][y];
                if (voxel.occupied && voxel.last_seen_occupied_ts >= min_ts) {
                    result.emplace_back(tile_key.x * kTileSize + x * kVoxelSize + kHalfVoxelSize,
                                        tile_key.y * kTileSize + y * kVoxelSize + kHalfVoxelSize);
                }
            }
        }
    }

    // TODO optimize (everything at once)
    if (range_limit > 0.) {
        Point car = {pose.x, pose.y};
        PointCloud filtered_result;
        for (Point& point : result) {
            if (Distance(car, point) <= range_limit) {
                filtered_result.push_back(point);
            }
        }
        result = filtered_result;
    }

    if (!absolute_coords) {
        CloudTransformation transformation = {pose.x, pose.y, pose.teta};
        PointCloud transformed_result = RevertTransformation(result, transformation);
        result = transformed_result;
    }

    return result;
}

std::unordered_map<size_t, Direction> sector_keys_reversed;

std::vector<Direction> GetWeightAcceptors(const Direction current_donor,
                                          const DirectionsWeights& all_donors) {
    std::vector<Direction> result;
    if (all_donors.empty()) {
        return result;
    }

    if (sector_keys_reversed.empty()) {
        for (auto it : kSectorKeys) {
            sector_keys_reversed[it.second] = it.first;
        }
    }

    size_t a_number = kSectorKeys.at(current_donor);

    size_t counter = 1;
    while (result.empty() && counter < 3) {
        Direction cw_sibling = sector_keys_reversed.at((a_number + counter) % 10);
        if (all_donors.find(cw_sibling) == all_donors.end()) {
            result.push_back(cw_sibling);
        }

        Direction ccw_sibling = sector_keys_reversed.at((a_number + 10 - counter) % 10);
        if (all_donors.find(ccw_sibling) == all_donors.end()) {
            result.push_back(ccw_sibling);
        }

        ++counter;
    }

    return result;
}

void Grid::CalculateTargetDistances() {
    std::queue<std::pair<TileKey, int>> queue;
    queue.emplace(target_tile_key_, 0);
    TileKey tile_key;
    TileKey next_tile;
    int distance;
    while (!queue.empty()) {
        auto it = queue.front();
        queue.pop();
        tile_key = it.first;
        distance = it.second;

        if (target_distances_.find(tile_key) != target_distances_.end()) {
            continue;
        }
        target_distances_[tile_key] = distance;

        for (int x = tile_key.x - 1; x <= tile_key.x + 1; ++x) {
            for (int y = tile_key.y - 1; y <= tile_key.y + 1; ++y) {
                if (x == tile_key.x && y == tile_key.y) {
                    continue;
                }
                next_tile.x = x;
                next_tile.y = y;
                if (free_tiles_.find(next_tile) != free_tiles_.end() &&
                    target_distances_.find(next_tile) == target_distances_.end()) {
                    queue.emplace(next_tile, distance + 1);
                }
            }
        }
    }
}

void Grid::UpdateTargetTile(double current_ts) {
    target_distances_ = {};
    if (free_tiles_.empty()) {
        return;
    }

    Pose current_pose = GetCurrentPose();
    TileKey car_tile_key = {
        static_cast<int>(std::floor(current_pose.x / kTileSize)),
        static_cast<int>(std::floor(current_pose.y / kTileSize)),
    };

    if (target_tile_key_ == car_tile_key || target_ts_ + kTargetTTL < current_ts) {
        Point car_point = {current_pose.x, current_pose.y};
        target_tile_key_ = *std::min_element(
            free_tiles_.begin(), free_tiles_.end(),
            [this, car_point](const TileKey& a, const TileKey& b) {
                if (tiles_[a].visited_ts_ < tiles_[b].visited_ts_) {
                    return true;
                } else if (tiles_[a].visited_ts_ == tiles_[b].visited_ts_) {
                    Point point_a = {a.x * kTileSize + kHalfTileSize,
                                     a.y * kTileSize + kHalfTileSize};
                    Point point_b = {b.x * kTileSize + kHalfTileSize,
                                     b.y * kTileSize + kHalfTileSize};
                    if (Distance(car_point, point_a) > Distance(car_point, point_b)) {
                        return true;
                    }
                }
                return false;
            });
        target_ts_ = current_ts;
    }

    if (free_tiles_.find(target_tile_key_) != free_tiles_.end()) {
        CalculateTargetDistances();
    }
}

double Grid::GetTileWeight(TileKey tile_key, double current_ts) const {
    if (tile_key == target_tile_key_) {
        return 5.;
    }

    return 0.;
}

std::vector<TileKeysToDirections> Grid::AssignTilesToDirections(
    CarInfo car_info, const std::unordered_set<TileKey>& nearby_tiles, TileKey target_tile) const {
    auto pose = GetCurrentPose();
    double shift = car_info.to_car_center;
    Point center_point = {
        pose.x + shift * cos(pose.teta),
        pose.y + shift * sin(pose.teta),
    };
    shift = car_info.to_car_center + car_info.turn_radius / 2.;
    Point front_point = {
        pose.x + shift * cos(pose.teta),
        pose.y + shift * sin(pose.teta),
    };
    shift = car_info.to_car_center - car_info.turn_radius / 2.;
    Point rear_point = {
        pose.x + shift * cos(pose.teta),
        pose.y + shift * sin(pose.teta),
    };
    shift = car_info.turn_radius;
    Point right_point = {
        pose.x + shift * cos(pose.teta - M_PI / 2),
        pose.y + shift * sin(pose.teta - M_PI / 2),
    };

    auto central_line = GetLinearCoefficients(front_point, rear_point);
    auto central_normal = NormalToLineInPoint(central_line, center_point);
    auto central_ns = GetTwoNRadiansLines(M_PI * 0.3, central_line, center_point);
    auto front_normal = NormalToLineInPoint(central_line, front_point);
    auto front_ns = GetTwoNRadiansLines(M_PI / 12., central_line, front_point);
    auto rear_normal = NormalToLineInPoint(central_line, rear_point);
    auto rear_ns = GetTwoNRadiansLines(M_PI / 12., central_line, rear_point);

    std::vector<TileKeysToDirections> result;
    TileKey tile_key;
    for (auto tile_pair : tiles_) {
        tile_key = tile_pair.first;
        if (!(tile_key == target_tile) && nearby_tiles.find(tile_key) == nearby_tiles.end()) {
            continue;
        }
        Point tile_center = {
            tile_key.x * kTileSize + kHalfTileSize,
            tile_key.y * kTileSize + kHalfTileSize,
        };

        bool left_side = OnOtherSide(central_line, tile_center, right_point);
        bool rear_side = OnOtherSide(central_normal, tile_center, front_point);

        bool f1 = OnOtherSide(central_ns.first, tile_center, front_point);
        bool f2 = OnOtherSide(central_ns.second, tile_center, front_point);
        bool r1 = OnOtherSide(central_ns.first, tile_center, rear_point);
        bool r2 = OnOtherSide(central_ns.second, tile_center, rear_point);

        Direction sector_key;
        if ((f1 && r2 && !f2 && !r1) || (!f1 && !r2 && f2 && r1)) {
            if (left_side) {
                if (rear_side) {
                    sector_key = {kMaxLeftDirection, kBackwardDirection};  // 3
                } else {
                    sector_key = {kMaxLeftDirection, kForwardDirection};  // 2
                }
            } else {
                if (rear_side) {
                    sector_key = {kMaxRightDirection, kBackwardDirection};  // 7
                } else {
                    sector_key = {kMaxRightDirection, kForwardDirection};  // 8
                }
            }
        } else if (rear_side) {
            if (OnOtherSide(rear_normal, tile_center, front_point) &&
                OnOtherSide(rear_ns.first, tile_center, front_point) &&
                OnOtherSide(rear_ns.second, tile_center, front_point)) {
                sector_key = {kStraightDirection, kBackwardDirection};  // 5
            } else if (left_side) {
                sector_key = {kABitLeftDirection, kBackwardDirection};  // 4
            } else {
                sector_key = {kABitRightDirection, kBackwardDirection};  // 6
            }
        } else {
            if (OnOtherSide(front_normal, tile_center, rear_point) &&
                OnOtherSide(front_ns.first, tile_center, rear_point) &&
                OnOtherSide(front_ns.second, tile_center, rear_point)) {
                sector_key = {kStraightDirection, kForwardDirection};  // 0
            } else if (left_side) {
                sector_key = {kABitLeftDirection, kForwardDirection};  // 1
            } else {
                sector_key = {kABitRightDirection, kForwardDirection};  // 9
            }
        }

        result.emplace_back(tile_key, sector_key, Distance(tile_center, center_point));
    }

    return result;
}

std::unordered_set<TileKey> Grid::GetNearbyTiles() {
    Pose current_pose = GetCurrentPose();
    TileKey car_tile_key = {
        static_cast<int>(std::floor(current_pose.x / kTileSize)),
        static_cast<int>(std::floor(current_pose.y / kTileSize)),
    };

    int distance_in_tiles = static_cast<int>(std::round(kUseForWeightsRadius / kTileSize));
    TileKey tile_key;
    std::unordered_set<TileKey> nearby_tiles;
    for (tile_key.x = car_tile_key.x - distance_in_tiles;
         tile_key.x <= car_tile_key.x + distance_in_tiles; ++tile_key.x) {
        for (tile_key.y = car_tile_key.y - distance_in_tiles;
             tile_key.y <= car_tile_key.y + distance_in_tiles; ++tile_key.y) {
            if (Distance(tile_key.x, tile_key.y, car_tile_key.x, car_tile_key.y) <=
                distance_in_tiles) {
                nearby_tiles.insert(tile_key);
            }
        }
    }

    for (auto it : nearby_tiles) {
        if (tiles_.find(it) == tiles_.end()) {
            tiles_[it] = Tile();
        }
    }

    return nearby_tiles;
}

std::pair<double, double> CartesianToRadial(double x, double y) {
    std::pair<double, double> result = {
        std::sqrt(x * x + y * y),
        std::atan2(y, x),
    };
    return result;
}

std::unordered_set<TileKey> GetFreeTiles(const PointCloud& self_points,
                                         const std::unordered_set<TileKey>& nearby_tiles,
                                         Pose pose) {
    std::unordered_map<size_t, double> sector_mins;
    for (int i = 0; i < kFreeTilesSectorsCount; ++i) {
        sector_mins[i] = kUseForWeightsRadius;
    }
    double angle_step = 2. * M_PI / static_cast<double>(kFreeTilesSectorsCount);
    double distance, angle;
    size_t index;
    for (auto point : self_points) {
        auto it = CartesianToRadial(point.x, point.y);
        distance = it.first;
        angle = it.second;
        if (angle < 0) {
            angle += 2. * M_PI;
        }
        index = static_cast<int>(std::floor(angle / angle_step));
        if (index >= kFreeTilesSectorsCount) {
            throw std::runtime_error("index out of range");
        }
        if (distance <= kWideSectorDistance) {
            size_t nearby_index;
            for (int i = static_cast<int>(index) - 1; i < static_cast<int>(index) + 1; ++i) {
                if (i < 0) {
                    nearby_index = static_cast<size_t>(i + kFreeTilesSectorsCount);
                } else if (i >= kFreeTilesSectorsCount) {
                    nearby_index = static_cast<size_t>(i - kFreeTilesSectorsCount);
                } else {
                    nearby_index = static_cast<size_t>(i);
                }
                if (distance < sector_mins[nearby_index]) {
                    sector_mins[nearby_index] = distance;
                }
            }
        } else if (distance < sector_mins[index]) {
            sector_mins[index] = distance;
        }
    }

    std::vector<TileKey> tile_keys;
    tile_keys.reserve(nearby_tiles.size());
    PointCloud tile_centers;
    tile_centers.reserve(nearby_tiles.size());
    for (auto tile_key : nearby_tiles) {
        tile_centers.emplace_back(tile_key.x * kTileSize + kHalfTileSize,
                                  tile_key.y * kTileSize + kHalfTileSize);
        tile_keys.push_back(tile_key);
    }
    CloudTransformation transformation = {pose.x, pose.y, pose.teta};
    PointCloud transformed_tile_centers = RevertTransformation(tile_centers, transformation);

    std::unordered_set<TileKey> result;
    for (int i = 0; i < static_cast<int>(transformed_tile_centers.size()); ++i) {
        auto it = CartesianToRadial(transformed_tile_centers[i].x, transformed_tile_centers[i].y);
        distance = it.first;
        angle = it.second;
        if (angle < 0) {
            angle += 2. * M_PI;
        }
        index = static_cast<int>(std::floor(angle / angle_step));
        if (index >= kFreeTilesSectorsCount) {
            throw std::runtime_error("index out of range");
        }
        if (distance < sector_mins[index]) {
            result.insert(tile_keys[i]);
        }
    }

    return result;
}

void MoveWeightsToNeighbourIfBlocked(const FreeDistances& free_distances, DirectionsWeights& result,
                                     CarInfo car_info, double target_distance) {
    double donor_distance = kWeightDonorDistanceMultiplier * car_info.turn_radius;
    double current_donor_distance = donor_distance;
    DirectionsWeights donors;
    DirectionsWeights all_acceptors;
    bool first_run = true;
    while (all_acceptors.empty() and current_donor_distance >= 0.1) {
        if (!first_run) {
            donors.clear();
            current_donor_distance /= 2.;
        }
        for (auto it : free_distances) {
            if (it.second <= current_donor_distance) {
                donors[it.first] = it.second;
            } else {
                all_acceptors[it.first] = it.second;
            }
        }
        first_run = false;
    }
    for (auto it : donors) {
        if (result[it.first] <= 0.) {
            continue;
        } else if (result[it.first] > kNoTilesButDistanceWeight &&
                   free_distances.at(it.first) >= target_distance) {
            continue;
        }
        double weight_to_transfer =
            result[it.first] * (donor_distance - it.second) / donor_distance;
        result[it.first] -= weight_to_transfer;
        auto current_acceptors = GetWeightAcceptors(it.first, donors);
        if (current_acceptors.empty()) {
            continue;
        } else if (current_acceptors.size() == 1) {
            result[current_acceptors[0]] += weight_to_transfer;
        } else {
            std::vector<double> distances;
            double sum_distance = 0.;
            for (auto an_acceptor : current_acceptors) {
                double dist = all_acceptors[an_acceptor] - donor_distance;
                distances.push_back(dist);
                sum_distance += dist;
            }
            for (size_t i = 0; i < current_acceptors.size(); ++i) {
                result[current_acceptors[i]] += weight_to_transfer * distances[i] / sum_distance;
            }
        }
    }
}

DirectionsWeights Grid::GetDirectionsWeights(double current_ts, CarInfo car_info) {
    DirectionsWeights result;
    for (auto it : kSectorKeys) {
        result[it.first] = 0.;
    }

    PointCloud nearby_points = GetSparsePointCloud(current_ts, kUseForWeightsRadius, false);
    auto free_distances = GetFreeDistances(nearby_points, car_info);
    if (std::all_of(free_distances.cbegin(), free_distances.cend(),
                    [](auto it) { return it.second == 0.; })) {
        return result;
    }

    auto nearby_tiles = GetNearbyTiles();
    free_tiles_ = GetFreeTiles(nearby_points, nearby_tiles, GetCurrentPose());
    UpdateTargetTile(current_ts);

    auto tile_to_direction = AssignTilesToDirections(car_info, free_tiles_, target_tile_key_);
    for (TileKeysToDirections& it : tile_to_direction) {
        double weight = GetTileWeight(it.tile_key, current_ts);
        double dist = it.distance;
        if (dist != 0.) {
            weight /= dist;
        }
        result[it.direction] += weight;
    }

    for (auto it : result) {
        if (it.second == 0. && free_distances[it.first] != 0.) {
            result[it.first] = kNoTilesButDistanceWeight;
        }
    }

    Point target_point = {target_tile_key_.x * kTileSize + kHalfTileSize,
                          target_tile_key_.y * kTileSize + kHalfTileSize};
    auto pose = GetCurrentPose();
    Point car_point = {pose.x, pose.y};
    double target_distance = Distance(car_point, target_point);
    MoveWeightsToNeighbourIfBlocked(free_distances, result, car_info, target_distance);

    return result;
}

PointCloud Grid::DebugGetFreeTileCenters(bool absolute) const {
    PointCloud tile_centers;
    tile_centers.reserve(free_tiles_.size());
    for (auto tile_key : free_tiles_) {
        tile_centers.emplace_back(tile_key.x * kTileSize + kHalfTileSize,
                                  tile_key.y * kTileSize + kHalfTileSize);
    }
    if (absolute) {
        return tile_centers;
    }

    auto pose = GetCurrentPose();
    CloudTransformation transformation = {pose.x, pose.y, pose.teta};
    return RevertTransformation(tile_centers, transformation);
}

Point Grid::DebugGetTargetPoint(bool absolute) const {
    Point result = {target_tile_key_.x * kTileSize + kHalfTileSize,
                    target_tile_key_.y * kTileSize + kHalfTileSize};

    if (absolute) {
        return result;
    }

    PointCloud points = {result};
    auto pose = GetCurrentPose();
    CloudTransformation transformation = {pose.x, pose.y, pose.teta};
    return RevertTransformation(points, transformation)[0];
}

void Grid::CleanUpGrid(double current_ts) {
    double min_ts = current_ts - kCleanUpFreeTileInterval;
    std::vector<TileKey> tiles_to_remove;
    bool has_active_voxels;
    for (auto tile_pair : tiles_) {
        Tile& tile = tile_pair.second;
        std::vector<std::vector<Voxel>>& voxels = tile.voxels_;
        has_active_voxels = false;
        for (size_t x = 0; x < voxels.size(); ++x) {
            for (size_t y = 0; y < voxels.size(); ++y) {
                Voxel& voxel = voxels[x][y];
                if (voxel.occupied && voxel.last_seen_occupied_ts >= min_ts) {
                    has_active_voxels = true;  // FIXME for big empty space
                    break;
                }
            }
            if (has_active_voxels) {
                break;
            }
        }
        if (!has_active_voxels) {
            tiles_to_remove.push_back(tile_pair.first);
        }
    }

    for (auto tile_key : tiles_to_remove) {
        tiles_.erase(tiles_.find(tile_key));
    }
}
