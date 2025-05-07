#ifndef NDT_TRACKER_H
#define NDT_TRACKER_H

#include "ndt.h"

const NDTSettings DEFAULT_SETTINGS{
    // int iter;
    1000,
    // double grid_step;
    1.5,
    // double grid_extent;
    8.,
    // double optim_step_x;
    0.1,
    // double optim_step_y;
    0.1,
    // double optim_step_theta;
    0.01,
    // double epsilon;
    0.0001,
    // matrix guess;
    Eigen::Matrix4f::Identity(),
};

const NDTSettings DEFAULT_SETTINGS_FAST{
    // int iter;
    25,
    // double grid_step;
    1.5,
    // double grid_extent;
    4.,
    // double optim_step_x;
    0.1,
    // double optim_step_y;
    0.1,
    // double optim_step_theta;
    0.1,
    // double epsilon;
    0.01,
    // matrix guess;
    Eigen::Matrix4f::Identity(),
};

PointType radial_to_cartesian(double distance, double angle);

double angle_to_a_range(double angle);

class Tracker {
public:
    void set_input(const std::vector<double> pose, const CloudPtr current_cloud);
    std::vector<double> get_pose();
    std::vector<double> get_error();

    void set_fast(bool fast) {
        fast_ = fast;
    }
private:
    bool fast_ = true;

    Eigen::Matrix4f prev_pose_ = Eigen::Matrix4f::Zero();
    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Zero();
    Eigen::Matrix4f error_ = Eigen::Matrix4f::Identity();

    CloudPtr prev_cloud_;
};

#endif