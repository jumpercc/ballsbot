#pragma once

#include <unordered_map>
#include <vector>
#include <deque>
#include <unordered_set>
#include <cstddef>
#include "common.h"
#include "point_cloud.h"

struct Pose {
    double x, y, teta;

    Pose(double x = 0., double y = 0., double teta = 0.) : x(x), y(y), teta(teta) {
    }
};

struct Voxel {
    bool occupied = false;
    double last_seen_occupied_ts = 0.;
    unsigned short points_count = 0;

    bool get_filtered_occupation() {
        if (this->occupied && this->points_count > max_points_to_ignore) {
            return true;
        }
        return false;
    }

private:
    const static unsigned char max_points_to_ignore = 1;
};

class Tile {
public:
    friend class Grid;
    Tile();
    void SetVoxelOccupied(int voxel_x, int voxel_y, double ts);
    void SetVisited(double ts);
    double GetVisitedTs() const {
        return visited_ts_;
    }
    std::vector<std::vector<Voxel>>& DebugGetVoxels() {
        return this->voxels_;
    }

private:
    std::vector<std::vector<Voxel>> voxels_;
    double visited_ts_;
};

struct TileKey {
    int x, y;

    TileKey(int x = 0, int y = 0) : x(x), y(y) {
    }

    bool operator==(const TileKey& other) const {
        return (this->x == other.x && this->y == other.y);
    }
};

namespace std {
template <>
struct hash<TileKey> {
    std::size_t operator()(TileKey const& s) const noexcept {
        std::size_t h1 = std::hash<int>{}(s.x);
        std::size_t h2 = std::hash<int>{}(s.y);
        return h1 ^ (h2 << 1);
    }
};
}  // namespace std

struct CarInfo {
    double to_car_center, to_pivot_center, turn_radius, car_width, car_length;
};

struct Direction {
    double steering, throttle;

    Direction(double steering = 0., double throttle = 0.) : steering(steering), throttle(throttle) {
    }

    bool operator==(const Direction& other) const {
        return (this->steering == other.steering && this->throttle == other.throttle);
    }
};

namespace std {
template <>
struct hash<Direction> {
    std::size_t operator()(Direction const& s) const noexcept {
        std::size_t h1 = std::hash<double>{}(s.steering);
        std::size_t h2 = std::hash<double>{}(s.throttle);
        return h1 ^ (h2 << 1);
    }
};
}  // namespace std

using DirectionsWeights = std::unordered_map<Direction, double>;

struct TileKeysToDirections {
    TileKey tile_key;
    Direction direction;
    double distance;

    TileKeysToDirections(TileKey a_tile_key, Direction a_direction, double a_distance)
        : tile_key(a_tile_key), direction(a_direction), distance(a_distance) {
    }
};

class Grid {
public:
    void UpdateGrid(PointCloud& cloud, Pose& pose, double ts);
    void UpdatePose(Pose& pose);

    void CleanUpGrid(double current_ts);

    std::unordered_map<TileKey, Tile>& DebugGetTiles() {
        return this->tiles_;
    }

    PointCloud GetSparsePointCloud(double current_ts, double range_limit, bool absolute_coords) const;

    DirectionsWeights GetDirectionsWeights(double current_ts, CarInfo car_info);

    Pose GetCurrentPose() const;
    std::vector<Pose> GetPoses() const;

    PointCloud DebugGetFreeTileCenters(bool absolute) const;
    Point DebugGetTargetPoint(bool absolute) const;

private:
    std::vector<TileKeysToDirections> AssignTilesToDirections(
        CarInfo car_info, const std::unordered_set<TileKey>& nearby_tiles, TileKey target_tile) const;
    double GetTileWeight(TileKey tile_key, double current_ts) const;
    std::unordered_set<TileKey> GetNearbyTiles();
    void UpdateTargetTile(double current_ts);
    void CalculateTargetDistances();

    std::unordered_map<TileKey, Tile> tiles_;
    size_t poses_to_keep_ = 250;
    std::deque<Pose> poses_;
    std::unordered_set<TileKey> free_tiles_;
    TileKey target_tile_key_;
    double target_ts_ = 0.;
    std::unordered_map<TileKey, int> target_distances_;
};

const double kMaxRightDirection = 1.;
const double kMaxLeftDirection = -1.;
const double kABitRightDirection = 0.5;
const double kABitLeftDirection = -0.5;
const double kStraightDirection = 0.;

const double kForwardDirection = 1.;
const double kBackwardDirection = -1.;

const std::unordered_map<Direction, size_t> kSectorKeys = {
    {{kStraightDirection, kForwardDirection}, 0},    //
    {{kABitLeftDirection, kForwardDirection}, 1},    //
    {{kMaxLeftDirection, kForwardDirection}, 2},     //
    {{kMaxLeftDirection, kBackwardDirection}, 3},    //
    {{kABitLeftDirection, kBackwardDirection}, 4},   //
    {{kStraightDirection, kBackwardDirection}, 5},   //
    {{kABitRightDirection, kBackwardDirection}, 6},  //
    {{kMaxRightDirection, kBackwardDirection}, 7},   //
    {{kMaxRightDirection, kForwardDirection}, 8},    //
    {{kABitRightDirection, kForwardDirection}, 9},   //
};
