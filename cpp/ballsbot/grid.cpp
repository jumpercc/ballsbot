#include <cmath>
#include <vector>
#include <queue>
#include <unordered_set>
#include <stdexcept>
#include <iostream>
#include "grid.h"
#include "geometry.h"
#include "point_cloud.h"

const double kVoxelSize = 0.1;
const size_t kVoxelsPerCell = 10;
const double kCellSize = kVoxelsPerCell * kVoxelSize;
const double kWeightDonorDistance = 1.2;

using Voxels = std::vector<std::vector<int>>;

struct VoxelIndexes {
    size_t x, y;

    VoxelIndexes(size_t x, size_t y) : x(x), y(y) {
    }

    bool operator==(const VoxelIndexes& other) const {
        return (this->x == other.x && this->y == other.y);
    }
};

namespace std {
template <>
struct hash<VoxelIndexes> {
    std::size_t operator()(VoxelIndexes const& s) const noexcept {
        std::size_t h1 = std::hash<size_t>{}(s.x);
        std::size_t h2 = std::hash<size_t>{}(s.y);
        return h1 ^ (h2 << 1);
    }
};
}  // namespace std

struct LineAndDistance {
    LinearCoefficients line;
    double d;

    LineAndDistance(LinearCoefficients line, double d) : line(line), d(d) {
    }
};

std::ostream& operator<<(std::ostream& out, Point const& data) {
    out << "(" << data.x << ", " << data.y << ")";
    return out;
}

std::ostream& operator<<(std::ostream& out, LinearCoefficients const& data) {
    out << "< " << data.a << "x + " << data.b << "y + " << data.c << " = 0 >";
    return out;
}

std::ostream& operator<<(std::ostream& out,
                         std::pair<LinearCoefficients, LinearCoefficients> const& data) {
    out << "[" << data.first << " and " << data.second << "]";
    return out;
}

void MarkHiddenVoxels(Voxels* voxels, size_t start_x, size_t start_y, size_t center_shift) {
    std::queue<VoxelIndexes> queue;
    queue.emplace(start_x, start_y);
    std::unordered_set<VoxelIndexes> seen;
    std::vector<LineAndDistance> lines;
    Point car_position = {double(center_shift), double(center_shift)};
    double w_max = sqrt(2 * 0.5 * 0.5);

    while (!queue.empty()) {
        auto indexes = queue.front();
        queue.pop();

        if (seen.find(indexes) != seen.end() || indexes.x >= voxels->size() ||
            indexes.y >= voxels->size()) {
            continue;
        }
        seen.insert(indexes);
        Point cell_center = {indexes.x + 0.5, indexes.y + 0.5};

        if (voxels->operator[](indexes.y)[indexes.x] > 0) {
            auto line = GetLinearCoefficients(car_position, cell_center);
            auto d = Distance(car_position, cell_center);
            lines.emplace_back(line, d);
        } else {
            auto d_big = Distance(car_position, cell_center);
            for (auto a_line : lines) {
                auto w_big = PointToLineDistance(cell_center, a_line.line);
                if (w_big * a_line.d / d_big <= w_max) {
                    voxels->operator[](indexes.y)[indexes.x] = -1;
                    break;
                }
            }
        }

        if (indexes.x < center_shift) {
            if (indexes.y < center_shift) {  // left down
                if (indexes.x > 0) {
                    queue.emplace(indexes.x - 1, indexes.y);
                    if (indexes.y > 0) {
                        queue.emplace(indexes.x - 1, indexes.y - 1);
                    }
                }
                if (indexes.y > 0) {
                    queue.emplace(indexes.x, indexes.y - 1);
                }
            } else {  // left up
                if (indexes.x > 0) {
                    queue.emplace(indexes.x - 1, indexes.y);
                    queue.emplace(indexes.x - 1, indexes.y + 1);
                }
                queue.emplace(indexes.x, indexes.y + 1);
            }
        } else {
            if (indexes.y < center_shift) {  // right down
                queue.emplace(indexes.x + 1, indexes.y);
                if (indexes.y > 0) {
                    queue.emplace(indexes.x + 1, indexes.y - 1);
                    queue.emplace(indexes.x, indexes.y - 1);
                }
            } else {  // right up
                queue.emplace(indexes.x + 1, indexes.y);
                queue.emplace(indexes.x + 1, indexes.y + 1);
                queue.emplace(indexes.x, indexes.y + 1);
            }
        }
    }
}

Voxels CloudToVoxels(const PointCloud& points, double half_width = 8.) {
    size_t size_in_voxels = 2 * int(round(half_width / kVoxelSize));
    Voxels result(size_in_voxels);
    for (size_t i = 0; i < size_in_voxels; ++i) {
        result[i].resize(size_in_voxels);
    }

    for (auto a_point : points) {
        int cell_x = int(round((a_point.x + half_width) / kVoxelSize));
        if (cell_x < 0) {
            cell_x = 0;
        } else if (cell_x >= int(size_in_voxels)) {
            cell_x = size_in_voxels - 1;
        }

        int cell_y = int(round((a_point.y + half_width) / kVoxelSize));
        if (cell_y < 0) {
            cell_y = 0;
        } else if (cell_y >= int(size_in_voxels)) {
            cell_y = size_in_voxels - 1;
        }

        ++result[cell_y][cell_x];
    }
    size_t center_shift = int(size_in_voxels / 2);
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            MarkHiddenVoxels(&result, center_shift + i - 1, center_shift + j - 1, center_shift);
        }
    }

    return result;
}

void FilterDisconnectedCells(FreeCells* cells) {
    std::queue<VoxelIndexes> queue;
    size_t center_shift = int(cells->size() / 2);
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            queue.emplace(center_shift + i - 1, center_shift + j - 1);
        }
    }

    std::unordered_set<VoxelIndexes> seen;
    while (!queue.empty()) {
        auto indexes = queue.front();
        queue.pop();

        if (seen.find(indexes) != seen.end() || indexes.x >= cells->size() ||
            indexes.y >= cells->size()) {
            continue;
        }
        seen.insert(indexes);

        if (indexes.x < center_shift) {
            if (indexes.y < center_shift) {  // left down
                if (cells->operator[](indexes.y)[indexes.x] &&
                    !(cells->operator[](indexes.y + 1)[indexes.x] ||
                      cells->operator[](indexes.y + 1)[indexes.x + 1] ||
                      cells->operator[](indexes.y)[indexes.x + 1])) {
                    cells->operator[](indexes.y)[indexes.x] = false;
                }
                if (indexes.x > 0) {
                    queue.emplace(indexes.x - 1, indexes.y);
                    if (indexes.y > 0) {
                        queue.emplace(indexes.x - 1, indexes.y - 1);
                    }
                }
                if (indexes.y > 0) {
                    queue.emplace(indexes.x, indexes.y - 1);
                }
            } else {  // left up
                if (cells->operator[](indexes.y)[indexes.x] &&
                    !(cells->operator[](indexes.y - 1)[indexes.x] ||
                      cells->operator[](indexes.y - 1)[indexes.x + 1] ||
                      cells->operator[](indexes.y)[indexes.x + 1])) {
                    cells->operator[](indexes.y)[indexes.x] = false;
                }
                if (indexes.x > 0) {
                    queue.emplace(indexes.x - 1, indexes.y);
                    queue.emplace(indexes.x - 1, indexes.y + 1);
                }
                queue.emplace(indexes.x, indexes.y + 1);
            }
        } else {
            if (indexes.y < center_shift) {  // right down
                if (cells->operator[](indexes.y)[indexes.x] &&
                    !(cells->operator[](indexes.y + 1)[indexes.x] ||
                      cells->operator[](indexes.y + 1)[indexes.x - 1] ||
                      cells->operator[](indexes.y)[indexes.x - 1])) {
                    cells->operator[](indexes.y)[indexes.x] = false;
                }
                queue.emplace(indexes.x + 1, indexes.y);
                if (indexes.y > 0) {
                    queue.emplace(indexes.x + 1, indexes.y - 1);
                    queue.emplace(indexes.x, indexes.y - 1);
                }
            } else {  // right up
                if (cells->operator[](indexes.y)[indexes.x] &&
                    !(cells->operator[](indexes.y - 1)[indexes.x] ||
                      cells->operator[](indexes.y - 1)[indexes.x - 1] ||
                      cells->operator[](indexes.y)[indexes.x - 1])) {
                    cells->operator[](indexes.y)[indexes.x] = false;
                }
                queue.emplace(indexes.x + 1, indexes.y);
                queue.emplace(indexes.x + 1, indexes.y + 1);
                queue.emplace(indexes.x, indexes.y + 1);
            }
        }
    }
}

FreeCellsResult CloudToFreeCells(const PointCloud& source_points, Pose pose) {
    size_t half_width = 4. * kCellSize;
    size_t size_in_cells = 2 * int(round((half_width / kCellSize)));
    FreeCells result(size_in_cells);
    for (size_t i = 0; i < size_in_cells; ++i) {
        result[i].resize(size_in_cells);
    }

    CloudTransformation transformation = {0., 0., pose.teta};
    auto points = ApplyTransformation(source_points, transformation);

    double square_part = 0.45;
    if (kCellSize > 1.) {
        square_part /= kCellSize * kCellSize;
    }
    auto voxels = CloudToVoxels(points, half_width);
    for (size_t cell_y = 0; cell_y < size_in_cells; ++cell_y) {
        for (size_t cell_x = 0; cell_x < size_in_cells; ++cell_x) {
            size_t voxels_count = 0;
            size_t free_voxels = 0;
            for (size_t voxel_y_o = 0; voxel_y_o < kVoxelsPerCell; ++voxel_y_o) {
                size_t voxel_y = voxel_y_o + cell_y * kVoxelsPerCell;
                if (voxel_y >= voxels.size()) {
                    break;
                }
                for (size_t voxel_x_o = 0; voxel_x_o < kVoxelsPerCell; ++voxel_x_o) {
                    size_t voxel_x = voxel_x_o + cell_x * kVoxelsPerCell;
                    if (voxel_x >= voxels.size()) {
                        break;
                    }
                    ++voxels_count;
                    if (voxels[voxel_y][voxel_x] == 0) {
                        ++free_voxels;
                    }
                }
            }
            if (voxels_count > 0 && double(free_voxels) / voxels_count >= square_part) {
                result[cell_y][cell_x] = true;
            }
        }
    }
    FilterDisconnectedCells(&result);

    int half_size_in_cells = int(size_in_cells / 2.);
    FreeCellsResult result_struct = {result, int(round(pose.x / kCellSize)) - half_size_in_cells,
                                     int(round(pose.y / kCellSize)) - half_size_in_cells};
    return result_struct;
}

std::pair<GridKey, bool> GetCarCell(double x, double y) {
    GridKey grid_key = {static_cast<int>(std::floor(x / kCellSize)),
                        static_cast<int>(std::floor(y / kCellSize))};
    bool inside_a_cell = Distance({(grid_key.x + 0.5) * kCellSize, (grid_key.y + 0.5) * kCellSize},
                                  {x, y}) <= kCellSize * 0.75 / 2.;
    std::pair<GridKey, bool> result = {grid_key, inside_a_cell};
    return result;
}

void Grid::UpdateGrid(PointCloud a_cloud, Pose a_pose) {
    auto free_cells = CloudToFreeCells(a_cloud, a_pose);
    auto shift_x = free_cells.shift_in_cells_x;
    auto shift_y = free_cells.shift_in_cells_y;

    for (size_t cell_y = 0; cell_y < free_cells.cells.size(); ++cell_y) {
        for (size_t cell_x = 0; cell_x < free_cells.cells.size(); ++cell_x) {
            GridKey grid_key = {int(cell_x) + shift_x, int(cell_y) + shift_y};
            if (free_cells.cells[cell_y][cell_x]) {
                if (this->grid_.find(grid_key) == this->grid_.end()) {
                    this->grid_[grid_key] = {};
                }
                this->grid_[grid_key].seen_at = this->counter_;
                ++this->grid_[grid_key].seen;
            } else if (this->grid_.find(grid_key) != this->grid_.end() &&
                       this->grid_[grid_key].was_at == -1 && this->grid_[grid_key].seen < 6) {
                --this->grid_[grid_key].seen;
            }
        }
    }

    auto car_cell = GetCarCell(a_pose.x, a_pose.y);
    if (car_cell.second) {
        auto car_grid_key = car_cell.first;
        if (this->grid_.find(car_grid_key) == this->grid_.end()) {
            this->grid_[car_grid_key] = {int(this->counter_), 0};
        }
        this->grid_[car_grid_key].was_at = int(this->counter_);
    }

    std::vector<GridKey> to_remove;
    for (auto it : this->grid_) {
        if (it.second.was_at != -1) {
            if (it.second.was_at + this->WAS_IN_MEMORY < this->counter_) {
                to_remove.push_back(it.first);
            }
        } else if (it.second.seen_at + this->SEEN_MEMORY < this->counter_) {
            to_remove.push_back(it.first);
        } else if (it.second.seen <= this->MIN_SEEN) {
            to_remove.push_back(it.first);
        }
    }
    for (auto it : to_remove) {
        this->grid_.erase(it);
    }

    ++this->counter_;
}

double Grid::GetCellWeight(GridInfo cell_info) const {
    if (cell_info.was_at != -1) {
        if (this->counter_ - cell_info.was_at < this->WAS_IN_MEMORY / 2.) {
            return 0.;
        } else {
            return (this->counter_ - cell_info.was_at) / (4. * this->WAS_IN_MEMORY);
        }
    } else {
        return 1.;
    }
}

double Grid::GetCellWeight(GridKey grid_key) const {
    auto it = this->grid_.find(grid_key);
    if (it != this->grid_.end()) {
        return this->GetCellWeight(it->second);
    } else {
        return 0.;
    }
}

std::vector<CellsToDirections> Grid::AssignCellsToDirections(Pose pose, CarInfo car_info) const {
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

    std::vector<CellsToDirections> result;
    for (auto it : this->grid_) {
        int cell_x = it.first.x;
        int cell_y = it.first.y;
        Point cell_center = {
            (cell_x + 0.5) * kCellSize,
            (cell_y + 0.5) * kCellSize,
        };

        bool left_side = OnOtherSide(central_line, cell_center, right_point);
        bool rear_side = OnOtherSide(central_normal, cell_center, front_point);

        bool f1 = OnOtherSide(central_ns.first, cell_center, front_point);
        bool f2 = OnOtherSide(central_ns.second, cell_center, front_point);
        bool r1 = OnOtherSide(central_ns.first, cell_center, rear_point);
        bool r2 = OnOtherSide(central_ns.second, cell_center, rear_point);

        Direction sector_key;
        if ((f1 && r2 && !f2 && !r1) || (!f1 && !r2 && f2 && r1)) {
            if (left_side) {
                if (rear_side) {
                    sector_key = {-1., -1.};  // 3
                } else {
                    sector_key = {-1., 1.};  // 2
                }
            } else {
                if (rear_side) {
                    sector_key = {1., -1.};  // 7
                } else {
                    sector_key = {1., 1.};  // 8
                }
            }
        } else if (rear_side) {
            if (OnOtherSide(rear_normal, cell_center, front_point) &&
                OnOtherSide(rear_ns.first, cell_center, front_point) &&
                OnOtherSide(rear_ns.second, cell_center, front_point)) {
                sector_key = {0., -1.};  // 5
            } else if (left_side) {
                sector_key = {-0.5, -1.};  // 4
            } else {
                sector_key = {0.5, -1.};  // 6
            }
        } else {
            if (OnOtherSide(front_normal, cell_center, rear_point) &&
                OnOtherSide(front_ns.first, cell_center, rear_point) &&
                OnOtherSide(front_ns.second, cell_center, rear_point)) {
                sector_key = {0., 1.};  // 0
            } else if (left_side) {
                sector_key = {-0.5, 1.};  // 1
            } else {
                sector_key = {0.5, 1.};  // 9
            }
        }

        result.emplace_back(it.first, sector_key, Distance(cell_center, center_point));
    }

    return result;
}

const std::unordered_map<Direction, size_t> kSectorKeys = {
    {{0., 1.}, 0},     //
    {{-0.5, 1.}, 1},   //
    {{-1., 1.}, 2},    //
    {{-1., -1.}, 3},   //
    {{-0.5, -1.}, 4},  //
    {{0., -1.}, 5},    //
    {{0.5, -1.}, 6},   //
    {{1., -1.}, 7},    //
    {{1., 1.}, 8},     //
    {{0.5, 1.}, 9},    //
};

std::unordered_map<size_t, Direction> sector_keys_reversed;

std::vector<Direction> GetWeightAcceptors(const Direction current_donor,
                                          const DirectionsWeights& all_donors) {
    if (sector_keys_reversed.empty()) {
        for (auto it : kSectorKeys) {
            sector_keys_reversed[it.second] = it.first;
        }
    }

    size_t a_number = kSectorKeys.at(current_donor);
    std::vector<Direction> result;

    Direction cw_sibling = sector_keys_reversed.at((a_number + 1) % 10);
    if (all_donors.find(cw_sibling) == all_donors.end()) {
        result.push_back(cw_sibling);
    }

    Direction ccw_sibling = sector_keys_reversed.at((a_number + 9) % 10);
    if (all_donors.find(ccw_sibling) == all_donors.end()) {
        result.push_back(ccw_sibling);
    }

    return result;
}

DirectionsWeights Grid::GetDirectionsWeights(Pose pose, CarInfo car_info,
                                             FreeDistances free_distances) const {
    DirectionsWeights result;
    for (auto it : kSectorKeys) {
        result[it.first] = 0.;
    }

    auto cell_to_direction = this->AssignCellsToDirections(pose, car_info);

    for (auto it : cell_to_direction) {
        double weight = this->GetCellWeight(this->grid_.at(it.grid_key));
        double dist = it.distance;
        if (dist != 0.) {
            weight /= dist;
        }
        result[it.direction] += weight;
    }

    DirectionsWeights donors;
    DirectionsWeights all_acceptors;
    for (auto it : free_distances) {
        if (it.second <= kWeightDonorDistance) {
            donors[it.first] = it.second;
        } else {
            all_acceptors[it.first] = it.second;
        }
    }
    for (auto it : donors) {
        if (result[it.first] <= 0.) {
            continue;
        }
        double weight_to_transfer =
            result[it.first] * (kWeightDonorDistance - it.second) / kWeightDonorDistance;
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
                double dist = all_acceptors[an_acceptor] - kWeightDonorDistance;
                distances.push_back(dist);
                sum_distance += dist;
            }
            for (size_t i = 0; i < current_acceptors.size(); ++i) {
                result[current_acceptors[i]] += weight_to_transfer * distances[i] / sum_distance;
            }
        }
    }

    return result;
}

std::vector<std::vector<size_t>> Grid::GetSectorsMap(Pose pose, CarInfo car_info,
                                                     size_t half_size) const {
    size_t a_size = 2 * half_size;
    std::vector<std::vector<size_t>> result(a_size);
    for (size_t i = 0; i < a_size; ++i) {
        result[i].resize(a_size);
    }

    auto cell_to_direction = this->AssignCellsToDirections(pose, car_info);
    for (auto it : cell_to_direction) {
        auto grid_key = it.grid_key;
        if (grid_key.x < -static_cast<int>(half_size) ||
            grid_key.y < -static_cast<int>(half_size)) {
            throw std::out_of_range("half_size is to small");
        }
        size_t x = grid_key.x + half_size;
        size_t y = grid_key.y + half_size;
        result[y][x] = kSectorKeys.at(it.direction) + 1;
    }

    return result;
}
