//
// Created by jumper on 13.06.2020.
//

#ifndef NORMAL_DISTRIBUTIONS_TRANSFORM_NDT_H
#define NORMAL_DISTRIBUTIONS_TRANSFORM_NDT_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt_2d.h>
#include <pcl/filters/approximate_voxel_grid.h>


using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;

struct NDTSettings
{
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

struct TransformationAndCloud
{
    Eigen::Matrix4f transformation;
    CloudPtr cloud;
    bool converged;
    double score;
};

TransformationAndCloud
get_transformation(const CloudPtr& target_cloud, const CloudPtr& input_cloud, const NDTSettings& settings)
{
    pcl::NormalDistributionsTransform2D<PointType, PointType> ndt;

    ndt.setMaximumIterations (settings.iter);
    ndt.setGridCentre (Eigen::Vector2f (0,0));
    ndt.setGridExtent (Eigen::Vector2f (settings.grid_extent, settings.grid_extent));
    ndt.setGridStep (Eigen::Vector2f (settings.grid_step, settings.grid_step));
    Eigen::Vector3d optim_step(settings.optim_step_x, settings.optim_step_y, settings.optim_step_theta);
    ndt.setOptimizationStepSize (optim_step);
    ndt.setTransformationEpsilon (settings.epsilon);

    ndt.setInputTarget (target_cloud);
    ndt.setInputSource (input_cloud);

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation (settings.guess_theta, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (settings.guess_x, settings.guess_y, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    CloudPtr output_cloud (new Cloud);
    ndt.align (*output_cloud, init_guess);

    TransformationAndCloud result = {
        ndt.getFinalTransformation(),
        output_cloud,
        ndt.hasConverged(),
        ndt.getFitnessScore(),
    };
    return result;
}

Eigen::Vector3d
transformation_matrix_to_xytheta(const Eigen::Matrix4f& transformation)
{
    const Eigen::Matrix3f initial_rot (transformation.block<3,3> (0,0));
    const Eigen::Vector3f rot_x (initial_rot*Eigen::Vector3f::UnitX ());
    const double z_rotation = std::atan2 (rot_x[1], rot_x[0]);
    Eigen::Vector3d xytheta_transformation (
            transformation (0,3),
            transformation (1,3),
            z_rotation
    );
    return xytheta_transformation;
}

std::vector<double>
get_transformation_vector (const CloudPtr& target_cloud, const CloudPtr& input_cloud, const NDTSettings& settings)
{
    TransformationAndCloud tnc = get_transformation(target_cloud, input_cloud, settings);
    auto xytheta_transformation = transformation_matrix_to_xytheta(tnc.transformation);
    tnc.cloud.reset();
    std::vector<double> result{
            xytheta_transformation.x(),
            xytheta_transformation.y(),
            xytheta_transformation.z(),
            tnc.converged ? 1. : 0.,
            tnc.score
    };
    return result;
}

#endif //NORMAL_DISTRIBUTIONS_TRANSFORM_NDT_H