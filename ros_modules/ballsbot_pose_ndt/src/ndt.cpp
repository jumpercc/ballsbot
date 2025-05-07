#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "ndt.h"

NDTResult get_transformation(const CloudPtr& target_cloud, const CloudPtr& input_cloud, const NDTSettings& settings) {
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

    CloudPtr output_cloud (new Cloud);
    ndt.align (*output_cloud, settings.guess);

    NDTResult result = {
        ndt.getFinalTransformation(),
        ndt.hasConverged(),
        ndt.getFitnessScore(),
    };
    return result;
}

Eigen::Vector3d transformation_matrix_to_xytheta(const Eigen::Matrix4f& transformation) {
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