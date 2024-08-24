#include <cmath>
#include <vector>
#include "ndt.h"
#include "ndt_tracker.h"

double ANGLE_FIX = 3.0660432755299887;  // FIXME use config

PointType radial_to_cartesian(double distance, double angle) {
    float x = distance * std::cos(angle);
    float y = distance * std::sin(angle);
    return {x, y, 0.0f};
}

double fix_angle(double my_angle, double angle_min, double angle_max) {
    my_angle -= ANGLE_FIX;
    if (my_angle < angle_min) {
        my_angle = angle_max + my_angle - angle_min;
    }
    else if (my_angle > angle_max) {
        my_angle = angle_min + my_angle - angle_max;
    }
    return my_angle;
}

double angle_to_a_range(double angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

void Tracker::set_input(const std::vector<double> pose, const CloudPtr current_cloud) {
    raw_x_ = pose[0];
    raw_y_ = pose[1];
    raw_teta_ = pose[2];

    if (prev_cloud_) {
        bool done = false;
        NDTResult transformation;
        Eigen::Matrix4f transformation_quat;

        NDTSettings current_settings = fast_ ? DEFAULT_SETTINGS_FAST : DEFAULT_SETTINGS;
        std::vector<std::vector<double>> gueses{
            {raw_x_ - prev_x_, raw_y_ - prev_y_, raw_teta_ - prev_teta_},
            {0., 0., 0.},
        };
        for(auto guess: gueses) {
            current_settings.guess_x = guess[0];
            current_settings.guess_y = guess[1];
            current_settings.guess_theta = guess[2];
            transformation = get_transformation(current_cloud, prev_cloud_, current_settings);
            if (transformation.converged && transformation.score < (fast_ ? 0.2 : 0.02)) {
                transformation_quat = transformation.transformation;
                done = true;
                break;
            }
        }

        if (!fast_ && !done) {
            std::vector<double> grid_steps{1.5, 1.0};
            std::vector<std::vector<double>> optim_steps{
                {0.1, 0.1, 0.02},
                {0.1, 0.1, 0.01},
            };
            for(auto grid_step: grid_steps) {
                current_settings = fast_ ? DEFAULT_SETTINGS_FAST : DEFAULT_SETTINGS;
                current_settings.grid_step = grid_step;
                gueses = {
                    {raw_x_ - prev_x_, raw_y_ - prev_y_, raw_teta_ - prev_teta_},
                    {0., 0., 0.},
                };
                for(auto optim_step: optim_steps) {
                    for(auto guess: gueses) {
                        current_settings.guess_x = guess[0];
                        current_settings.guess_y = guess[1];
                        current_settings.guess_theta = guess[2];
                        transformation = get_transformation(current_cloud, prev_cloud_, current_settings);
                        if (transformation.converged && transformation.score < 0.5) {
                            transformation_quat = transformation.transformation;
                            done = true;
                            break;
                        }
                    }
                    if (done) {
                        if (transformation.score < 0.06) {
                            break;
                        }
                        else {
                            done = false;
                            auto better_diff = transformation_matrix_to_xytheta(transformation_quat);
                            gueses[0][0] = better_diff[0];
                            gueses[0][1] = better_diff[1];
                            gueses[0][2] = better_diff[2];
                        }
                    }
                }
                if (done && transformation.score < 0.06) {
                    break;
                }
            }
            if (!done || transformation.score > 0.39) {
                done = false; // keep raw pose value
            }
        }

        if (done) {
            auto fixed_diff = transformation_matrix_to_xytheta(transformation_quat);
            x_error_ += fixed_diff[0] - (raw_x_ - prev_x_);
            y_error_ += fixed_diff[1] - (raw_y_ - prev_y_);
            teta_error_ += fixed_diff[2] - (raw_teta_ - prev_teta_);
            teta_error_ = angle_to_a_range(teta_error_);
        }
        else {
            // TODO notify client somehow
        }
    }

    prev_cloud_ = current_cloud;
    prev_x_ = raw_x_;
    prev_y_ = raw_y_;
    prev_teta_ = raw_teta_;
}
