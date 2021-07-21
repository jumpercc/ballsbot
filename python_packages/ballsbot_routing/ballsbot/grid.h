#pragma once

#include <unordered_map>
#include <vector>
#include "common.h"
#include "point_cloud.h"

struct Pose {
    double x, y, teta;

    Pose(double x = 0., double y = 0., double teta = 0.) : x(x), y(y), teta(teta) {
    }
};

struct GridKey {
    int x, y;

    GridKey(int x = 0, int y = 0) : x(x), y(y) {
    }

    bool operator==(const GridKey& other) const {
        return (this->x == other.x && this->y == other.y);
    }
};

namespace std {
template <>
struct hash<GridKey> {
    std::size_t operator()(GridKey const& s) const noexcept {
        std::size_t h1 = std::hash<int>{}(s.x);
        std::size_t h2 = std::hash<int>{}(s.y);
        return h1 ^ (h2 << 1);
    }
};
}  // namespace std

struct CarInfo {
    double to_car_center, turn_radius;
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

struct GridInfo {
    int seen_at, was_at, seen;

    GridInfo() : GridInfo(-1, 0) {
    }

    GridInfo(int seen_at, int seen, int was_at = -1)
        : seen_at(seen_at), was_at(was_at), seen(seen) {
    }

    bool operator==(const GridInfo& other) const {
        return (this->seen_at == other.seen_at && this->was_at == other.was_at &&
                this->seen == other.seen);
    }
};

using FreeCells = std::vector<std::vector<bool>>;

struct FreeCellsResult {
    FreeCells cells;
    int shift_in_cells_x, shift_in_cells_y;
};

FreeCellsResult CloudToFreeCells(const PointCloud& source_points, Pose pose);

struct CellsToDirections {
    GridKey grid_key;
    Direction direction;
    double distance;

    CellsToDirections(GridKey grid_key, Direction direction, double distance)
        : grid_key(grid_key), direction(direction), distance(distance) {
    }
};

using FreeDistances = std::vector<std::pair<Direction, double>>;

class Grid {
public:
    const size_t SEEN_MEMORY = 100;  // 50 -> 12.5 sec when 4 fps
    const size_t WAS_IN_MEMORY = 200;
    const int MIN_SEEN = 0;

    Grid() : counter_(0), grid_() {
    }

    void UpdateGrid(PointCloud a_cloud, Pose a_pose);

    double GetCellWeight(GridKey grid_key) const;

    DirectionsWeights GetDirectionsWeights(Pose pose, CarInfo car_info,
                                           FreeDistances free_distances) const;

    std::unordered_map<GridKey, GridInfo> DebugGetGrid() const {
        return this->grid_;
    }

    std::vector<std::vector<size_t>> GetSectorsMap(Pose pose, CarInfo car_info,
                                                   size_t half_size) const;

private:
    double GetCellWeight(GridInfo cell_info) const;
    std::vector<CellsToDirections> AssignCellsToDirections(Pose pose, CarInfo car_info) const;

    size_t counter_;
    std::unordered_map<GridKey, GridInfo> grid_;
};
