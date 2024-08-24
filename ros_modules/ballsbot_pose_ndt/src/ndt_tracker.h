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
    // double guess_x;
    0.,
    // double guess_y;
    0.,
    // double guess_theta;
    0.,
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
    // double guess_x;
    0.,
    // double guess_y;
    0.,
    // double guess_theta;
    0.,
};

PointType radial_to_cartesian(double distance, double angle);

double fix_angle(double my_angle, double angle_min, double angle_max);

double angle_to_a_range(double angle);

class Tracker {
public:
    void set_input(const std::vector<double> pose, const CloudPtr current_cloud);

    double get_x() {
        return raw_x_ + x_error_;
    }

    double get_y() {
        return raw_y_ + y_error_;
    }

    double get_teta() {
        return angle_to_a_range(raw_teta_ + teta_error_);
    }

    double get_x_error() {
        return x_error_;
    }

    double get_y_error() {
        return y_error_;
    }

    double get_teta_error() {
        return teta_error_;
    }

    void set_fast(bool fast) {
        fast_ = fast;
    }
private:
    bool fast_ = true;

    double raw_x_ = 0.0;
    double raw_y_ = 0.0;
    double raw_teta_ = 0.0;

    double x_error_ = 0.0;
    double y_error_ = 0.0;
    double teta_error_ = 0.0;

    double prev_x_ = 0.0;
    double prev_y_ = 0.0;
    double prev_teta_ = 0.0;

    CloudPtr prev_cloud_;
};

#endif