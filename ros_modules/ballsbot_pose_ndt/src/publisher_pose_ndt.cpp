#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "ballsbot_pose/Pose.h"
#include "ballsbot_pose_ndt/FixedPose.h"
#include "sensor_msgs/LaserScan.h"
#include "ndt.h"

double ANGLE_FIX = 3.0660432755299887;  // FIXME use config

using PosePtr = ballsbot_pose::Pose::ConstPtr;
using LidarPtr = sensor_msgs::LaserScan::ConstPtr;

PosePtr current_pose_msg;
bool pose_used = false;
LidarPtr current_lidar_msg;
bool lidar_used = false;

void PoseCallback(const PosePtr &msg) {
    current_pose_msg = msg;
    pose_used = false;
}

void LidarCallback(const LidarPtr &msg) {
    current_lidar_msg = msg;
    lidar_used = false;
}

const NDTSettings DEFAULT_SETTINGS{
    // int iter;
    1000,
    // double grid_step;
    1.5,
    // double grid_extent;
    20.,
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

CloudPtr get_cloud_from_message(const LidarPtr &lidar_msg) {
    auto result = CloudPtr(new Cloud(0, 1));
    double angle = lidar_msg->angle_min;
    size_t i = 0;
    for (auto value: lidar_msg->intensities) {
        if (value > 0) {
            result->push_back(radial_to_cartesian(
                lidar_msg->ranges[i],
                fix_angle(angle, lidar_msg->angle_min, lidar_msg->angle_max)
            ));
        }
        angle += lidar_msg->angle_increment;
        i++;
    }
    return result;
}

class Tracker {
public:
    void set_input(const PosePtr &pose_msg, const LidarPtr &lidar_msg) {
        prev_x_ = x_;
        prev_y_ = y_;
        prev_teta_ = teta_;

        x_ = pose_msg->x;
        y_ = pose_msg->y;
        teta_ = pose_msg->teta;

        auto current_cloud = get_cloud_from_message(lidar_msg);
        if (prev_cloud_) {
            bool done = false;
            TransformationAndCloud transformation;
            Eigen::Matrix4f transformation_quat;

            NDTSettings current_settings = DEFAULT_SETTINGS;
            std::vector<std::vector<double>> gueses{
                {x_ - prev_x_, y_ - prev_y_, teta_ - prev_teta_},
                {0., 0., 0.},
            };
            for(auto guess: gueses) {
                current_settings.guess_x = guess[0];
                current_settings.guess_y = guess[1];
                current_settings.guess_theta = guess[2];
                transformation = get_transformation(current_cloud, prev_cloud_, current_settings);
                if (transformation.converged && transformation.score < 0.02) {
                    transformation_quat = transformation.transformation;
                    done = true;
                    break;
                }
            }

            if (!done) {
                std::vector<double> grid_steps{1.5, 1.0};
                std::vector<std::vector<double>> optim_steps{
                    {0.1, 0.1, 0.02},
                    {0.1, 0.1, 0.01},
                };
                for(auto grid_step: grid_steps) {
                    current_settings = DEFAULT_SETTINGS;
                    current_settings.grid_step = grid_step;
                    gueses = {
                        {x_ - prev_x_, y_ - prev_y_, teta_ - prev_teta_},
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
                                auto better_pose = transformation_matrix_to_xytheta(transformation_quat);
                                gueses[0][0] = better_pose[0];
                                gueses[0][1] = better_pose[1];
                                gueses[0][2] = better_pose[2];
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
                auto fixed_pose = transformation_matrix_to_xytheta(transformation_quat);
                x_ = fixed_pose[0];
                y_ = fixed_pose[1];
                teta_ = fixed_pose[2];
            }
            else {
                ROS_INFO("failed to fix a pose");
            }
        }
        prev_cloud_ = current_cloud;
    }

    double get_x() {
        return x_;
    }

    double get_y() {
        return y_;
    }

    double get_teta() {
        return teta_;
    }
private:
    double x_ = 0.0;
    double y_ = 0.0;
    double teta_ = 0.0;

    double prev_x_ = 0.0;
    double prev_y_ = 0.0;
    double prev_teta_ = 0.0;

    CloudPtr prev_cloud_;
};

int main(int argc, char *argv[]) {
    Tracker tracker;
    ros::init(argc, argv, "ballsbot_pose_ndt");
    ros::NodeHandle n;

    ros::Subscriber pose_subscriber = n.subscribe("pose", 1, PoseCallback);
    ros::Subscriber lidar_subscriber = n.subscribe("scan", 1, LidarCallback);

    ballsbot_pose_ndt::FixedPose output_msg;
    auto pose_pub = n.advertise<ballsbot_pose_ndt::FixedPose>("pose_ndt", 1);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate rate(5);
    while (ros::ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();

        if (current_pose_msg.get() != nullptr && current_lidar_msg.get() != nullptr && !pose_used && !lidar_used) {
            pose_used = true;
            lidar_used = true;
            tracker.set_input(current_pose_msg, current_lidar_msg);

            output_msg.header.stamp = current_time;
            output_msg.pose_ts = current_pose_msg->header.stamp;
            output_msg.x = tracker.get_x();
            output_msg.y = tracker.get_y();
            output_msg.teta = tracker.get_teta();
            output_msg.x_raw = current_pose_msg->x;
            output_msg.y_raw = current_pose_msg->y;
            output_msg.teta_raw = current_pose_msg->teta;
            pose_pub.publish(output_msg);

            last_time = current_time;
        }

        rate.sleep();
    }

    return 0;
}
