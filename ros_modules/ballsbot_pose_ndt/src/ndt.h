//
// Created by jumper on 13.06.2020.
//

#ifndef NORMAL_DISTRIBUTIONS_TRANSFORM_NDT_H
#define NORMAL_DISTRIBUTIONS_TRANSFORM_NDT_H

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;

struct NDTSettings {
    int iter;
    double grid_step;
    double grid_extent;
    double optim_step_x;
    double optim_step_y;
    double optim_step_theta;
    double epsilon;
    double guess_x;
    double guess_y;
    double guess_theta;
};

struct NDTResult {
    Eigen::Matrix4f transformation;
    bool converged;
    double score;
};

NDTResult get_transformation(const CloudPtr& target_cloud, const CloudPtr& input_cloud, const NDTSettings& settings);

Eigen::Vector3d transformation_matrix_to_xytheta(const Eigen::Matrix4f& transformation);

#endif //NORMAL_DISTRIBUTIONS_TRANSFORM_NDT_H