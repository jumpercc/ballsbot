#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "ballsbot_pose/Pose.h"
#include "ballsbot_pose_ndt/FixedPose.h"
#include "sensor_msgs/LaserScan.h"
#include "ndt.h"
#include "ndt_tracker.h"

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

CloudPtr get_cloud_from_message(const LidarPtr &lidar_msg) {
    auto result = CloudPtr(new Cloud());
    double angle = lidar_msg->angle_min;
    size_t i = 0;
    for (auto value: lidar_msg->intensities) {
        if (value > 0 && lidar_msg->ranges[i] < 5.) {
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

int main(int argc, char *argv[]) {
    Tracker tracker;
    size_t ticks_count = 0;
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
            ticks_count = (ticks_count + 1) % 15;
            if (ticks_count == 0) {
                std::vector<double> current_pose {current_pose_msg->x, current_pose_msg->y, current_pose_msg->teta};
                auto current_cloud = get_cloud_from_message(current_lidar_msg);
                tracker.set_input(current_pose, current_cloud);
            }

            output_msg.header.stamp = current_time;
            output_msg.pose_ts = current_pose_msg->header.stamp;
            output_msg.x = tracker.get_x();
            output_msg.y = tracker.get_y();
            output_msg.teta = tracker.get_teta();
            output_msg.x_error = tracker.get_x_error();
            output_msg.y_error = tracker.get_y_error();
            output_msg.teta_error = tracker.get_teta_error();
            pose_pub.publish(output_msg);

            last_time = current_time;
        }

        rate.sleep();
    }

    return 0;
}
