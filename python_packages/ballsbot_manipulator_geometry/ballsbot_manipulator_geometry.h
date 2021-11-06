#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <vector>
#include <stdexcept>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;

PointType operator+(const PointType& left, const PointType& right) {
    PointType result{
        left.x + right.x,
        left.y + right.y,
        left.z + right.z,
    };
    return result;
}

PointType operator-(const PointType& left, const PointType& right) {
    PointType result{
        left.x - right.x,
        left.y - right.y,
        left.z - right.z,
    };
    return result;
}

enum AxisName { kX, kY, kZ };

auto RotationTypeByAxis(enum AxisName name) {
    switch (name) {
        case kX:
            return Eigen::Vector3f::UnitX();
        case kY:
            return Eigen::Vector3f::UnitY();
        case kZ:
            return Eigen::Vector3f::UnitZ();
    }
    throw std::runtime_error("unknown axis name");
}

std::vector<PointType> ApplyRotations(const std::vector<PointType>& default_points,
                                      const std::vector<PointType>& rotations) {
    auto result = default_points;

    Eigen::Translation3f dummy_translation(0, 0, 0);
    float angle;
    auto axis_name = kX;

    size_t rotations_i = rotations.size();
    for (int i = default_points.size() - 2; i >= 0; --i) {
        --rotations_i;
        auto rotation_center = default_points[i];
        auto need_rotation = true;
        auto claw = (rotations_i == rotations.size() - 1);

        if (rotations[rotations_i].x != 0.) {
            angle = rotations[rotations_i].x;
            axis_name = kX;
        } else if (rotations[rotations_i].y != 0.) {
            angle = rotations[rotations_i].y;
            axis_name = kY;
        } else if (rotations[rotations_i].z != 0.) {
            angle = rotations[rotations_i].z;
            axis_name = kZ;
        } else {
            need_rotation = false;
        }

        if (need_rotation) {
            int part_limit = claw ? -1 : 1;
            for (int part_i = 1; part_i >= part_limit; part_i -= 2) {
                angle *= static_cast<float>(part_i);
                Eigen::AngleAxisf rotation_transformation(angle, RotationTypeByAxis(axis_name));
                Eigen::Matrix4f transform_matrix =
                    (rotation_transformation * dummy_translation).matrix();
                Cloud input_cloud;
                for (size_t j = i + 1; j != result.size(); ++j) {
                    if (claw) {
                        input_cloud.push_back(default_points[j] - rotation_center);
                    } else {
                        input_cloud.push_back(result[j] - rotation_center);
                    }
                }
                Cloud output_cloud = input_cloud;
                transformPointCloud(input_cloud, output_cloud, transform_matrix);
                for (size_t j = i + 1; j != result.size(); ++j) {
                    auto new_point = output_cloud.points[j - i - 1] + rotation_center;
                    if (claw && part_i == -1) {
                        result.push_back(new_point);
                        break;
                    } else {
                        result[j] = new_point;
                    }
                }
            }
        } else if (claw) {
            result.push_back(result[result.size() - 1]);
        }
    }

    return result;
}
