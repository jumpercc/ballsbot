#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include "ndt.h"
#include "ndt_tracker.h"

PointType radial_to_cartesian(double distance, double angle) {
    float x = distance * std::cos(angle);
    float y = distance * std::sin(angle);
    return {x, y, 0.0f};
}

double angle_to_a_range(double angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

Eigen::Matrix4f matrix_from_xytheta(double x, double y, double teta) {
    Eigen::AngleAxisf a_rotation(teta, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f a_translation(x, y, 0.);
    return (a_translation * a_rotation).matrix();
}

std::vector<double> Tracker::get_pose() {
    auto raw_result = transformation_matrix_to_xytheta(current_pose_);
    std::vector<double> result = {raw_result[0], raw_result[1], angle_to_a_range(raw_result[2])};
    return result;
}

std::vector<double> Tracker::get_error() {
    auto raw_result = transformation_matrix_to_xytheta(error_);
    std::vector<double> result = {raw_result[0], raw_result[1], angle_to_a_range(raw_result[2])};
    return result;
}

void print_matrix(Eigen::Matrix4f a_matrix, std::string label) {
    std::cerr << label << std::endl << a_matrix << std::endl;
}

void print_xyteta(Eigen::Matrix4f a_matrix, std::string label) {
    auto raw_result = transformation_matrix_to_xytheta(a_matrix);
    std::vector<double> result = {raw_result[0], raw_result[1], angle_to_a_range(raw_result[2])};
    std::cerr << label << std::endl << "x = " << result[0] << ", y = " << result[1] << ", teta = " << result[2] << std::endl;
}

void Tracker::set_input(const std::vector<double> pose, const CloudPtr current_cloud) {
    auto current_pose_raw = matrix_from_xytheta(pose[0], pose[1], pose[2]);
    // print_xyteta(current_pose_raw, "current_pose_raw");
    if (prev_cloud_) {
        bool done = false;
        NDTResult transformation;
        Eigen::Matrix4f transformation_quat;

        auto first_guess = current_pose_raw.inverse() * prev_pose_;

        NDTSettings current_settings = fast_ ? DEFAULT_SETTINGS_FAST : DEFAULT_SETTINGS;
        std::vector<Eigen::Matrix4f> gueses{
            first_guess,
            Eigen::Matrix4f::Identity(),
        };
        for(auto guess: gueses) {
            current_settings.guess = guess;
            transformation = get_transformation(current_cloud, prev_cloud_, current_settings);
            // std::cerr << "MRKR1 " << transformation.score << std::endl;
            if (transformation.converged && transformation.score < (fast_ ? 0.21 : 0.021)) {
                transformation_quat = transformation.transformation;
                done = true;
                break;
            }
        }

        if (done) {
            transformation_quat = transformation_quat.inverse();
            // print_matrix(transformation_quat, "transformation");

            auto error = current_pose_raw.inverse() * prev_pose_ * transformation_quat;
            // print_xyteta(error, "step error");

            // print_xyteta(error_, "total error before");
            error_ = error * error_;
            // print_xyteta(error_, "total error after");

            current_pose_ = current_pose_raw * error_;
            // print_xyteta(current_pose_, "current_pose_ after");

            std::cerr << "aligned with a score of " << transformation.score << std::endl;
        }
        else {
            std::cerr << "failed to align clouds (" << pose[0] << ", " << pose[1] << ", " << pose[2] << ")" << std::endl;
            current_pose_ = current_pose_raw * error_;
        }
    }
    else {
        current_pose_ = current_pose_raw;
    }

    prev_cloud_ = current_cloud;
    prev_pose_ = current_pose_raw;
}
