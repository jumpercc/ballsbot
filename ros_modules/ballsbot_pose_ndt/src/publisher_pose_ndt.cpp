#include <cmath>
#include "ros/ros.h"
#include "ballsbot_pose/Pose.h"
#include "ballsbot_pose_ndt/FixedPose.h"
#include "sensor_msgs/LaserScan.h"
#include "ndt.h"

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

class Tracker {
public:
    void set_input(const PosePtr &pose_msg, const LidarPtr &lidar_msg) {
        // FIXME
        x_ = pose_msg->x;
        y_ = pose_msg->y;
        teta_ = pose_msg->teta;
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
