#include <catch.hpp>
#include "ballsbot_manipulator_geometry.h"

namespace std {
bool operator==(const PointType left, const PointType right) {
    return (left.x == right.x && left.y == right.y && left.z == right.z);
}
};  // namespace std

TEST_CASE("ballsbot_manipulator/ApplyRotations no rotations") {
    std::vector<PointType> default_points = {
        {148.5, -100.0, -56.5}, {129.5, -105.5, -56.5}, {-20.0, -124.5, -56.5},
        {168.0, -130.5, -55.5}, {223.0, -130.5, -55.5},
    };
    std::vector<PointType> rotations = {
        {0., 0., 0.},
        {0., 0., 0.},
        {0., 0., 0.},
        {0., 0., 0.},
    };
    std::vector<PointType> expected_points = {
        {148.5, -100.0, -56.5}, {129.5, -105.5, -56.5}, {-20.0, -124.5, -56.5},
        {168.0, -130.5, -55.5}, {223.0, -130.5, -55.5}, {223.0, -130.5, -55.5},
    };

    std::vector<PointType> result_points = ApplyRotations(default_points, rotations);
    REQUIRE(result_points == expected_points);
}

TEST_CASE("ballsbot_manipulator/ApplyRotations slight rotation") {
    std::vector<PointType> default_points = {
        {148.5, -100.0, -56.5}, {129.5, -105.5, -56.5}, {-20.0, -124.5, -56.5},
        {168.0, -130.5, -55.5}, {223.0, -130.5, -55.5},
    };
    std::vector<PointType> rotations = {
        {0., 0., 0.1},
        {0., 0.12, 0.},
        {0., -0.1, 0.},
        {0.1, 0., 0.},
    };

    std::vector<PointType> result_points = ApplyRotations(default_points, rotations);
    REQUIRE(result_points.size() == default_points.size() + 1);
}
