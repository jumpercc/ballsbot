#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "point_cloud.h"

using Eigen::MatrixXd;

PointCloud ApplyTransformation(const PointCloud& source_cloud, CloudTransformation transformation) {
    MatrixXd rotate(2, 2);
    rotate(0, 0) = cos(transformation.dteta);
    rotate(0, 1) = -sin(transformation.dteta);
    rotate(1, 0) = -rotate(0, 1);
    rotate(1, 1) = rotate(0, 0);

    MatrixXd move(2, source_cloud.size());
    MatrixXd cloud(2, source_cloud.size());
    for (size_t i = 0; i < source_cloud.size(); ++i) {
        cloud(0, i) = source_cloud[i].x;
        cloud(1, i) = source_cloud[i].y;
        move(0, i) = transformation.dx;
        move(1, i) = transformation.dy;
    }

    auto result_matrix = rotate * cloud + move;

    PointCloud result;
    result.reserve(source_cloud.size());
    for (size_t i = 0; i < source_cloud.size(); ++i) {
        result.emplace_back(result_matrix(0, i), result_matrix(1, i));
    }

    return result;
}

PointCloud RevertTransformation(const PointCloud& source_cloud, CloudTransformation transformation) {
    MatrixXd rotate(2, 2);
    rotate(0, 0) = cos(transformation.dteta);
    rotate(0, 1) = -sin(transformation.dteta);
    rotate(1, 0) = -rotate(0, 1);
    rotate(1, 1) = rotate(0, 0);

    MatrixXd move(2, source_cloud.size());
    MatrixXd cloud(2, source_cloud.size());
    for (size_t i = 0; i < source_cloud.size(); ++i) {
        cloud(0, i) = source_cloud[i].x;
        cloud(1, i) = source_cloud[i].y;
        move(0, i) = transformation.dx;
        move(1, i) = transformation.dy;
    }

    auto result_matrix = rotate.inverse() * (cloud - move);

    PointCloud result;
    result.reserve(source_cloud.size());
    for (size_t i = 0; i < source_cloud.size(); ++i) {
        result.emplace_back(result_matrix(0, i), result_matrix(1, i));
    }

    return result;
}
