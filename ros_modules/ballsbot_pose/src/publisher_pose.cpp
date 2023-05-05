#include <cmath>
#include "ros/ros.h"
#include "ballsbot_pose/Pose.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "ballsbot_imu/ImuState.h"
#include "ballsbot_wheel_odometry/OdometryState.h"

struct Pose {
    double x, y, teta, ts;
    bool first_time;

    Pose() : x(0.), y(0.), teta(0.), ts(0.), first_time(true) {
    }
};

struct Point {
    Point(double x_value, double y_value) : x(x_value), y(y_value) {
    }

    Point() : x(0.), y(0.) {
    }

    double x, y;
};

using ImuPtr = ballsbot_imu::ImuState::ConstPtr;
using OdometryPtr = ballsbot_wheel_odometry::OdometryState::ConstPtr;

ImuPtr current_imu_msg;
OdometryPtr current_odometry_msg;

void ImuCallback(const ImuPtr &msg) {
    current_imu_msg = msg;
}

void OdometryCallback(const OdometryPtr &msg) {
    current_odometry_msg = msg;
}

struct DxDyState {
    double teta, w_z, dt;
    bool first_time;

    DxDyState() : teta(0.), w_z(0.), dt(0.), first_time(true) {
    }

    DxDyState(double teta_value, double w_z_value, double dt_value, bool first_time_value) :
            teta(teta_value), w_z(w_z_value), dt(dt_value), first_time(first_time_value) {
    }
};

class Tracker {
public:
    Pose GetCurrentPose(const ImuPtr &imu_msg, const OdometryPtr &odometry_msg) {
        Pose current_pose;
        current_pose.first_time = false;
        current_pose.ts = imu_msg->header.stamp.toSec();
        current_pose.teta = imu_msg->teta;
        current_pose.x = prev_pose.x;
        current_pose.y = prev_pose.y;

        if (!this->prev_pose.first_time) {
            double dt = current_pose.ts - this->prev_pose.ts;
            if (dt != 0) {
                auto dx_dy = GetDxDy(dt, imu_msg->teta, imu_msg, odometry_msg);
                current_pose.x += dx_dy.x;
                current_pose.y += dx_dy.y;
            }
        }

        this->prev_pose = current_pose;
        return current_pose;
    }

private:
    DxDyState prev_dx_dy;
    Pose prev_pose;

    Point GetDxDy(double dt, double teta, const ImuPtr &imu_msg, const OdometryPtr &odometry_msg) {
        Point result = {0., 0.};
        DxDyState current(teta, imu_msg->w_z, dt, false);
        if (!this->prev_dx_dy.first_time) {
            double dteta = teta - this->prev_dx_dy.teta;
            if (dteta > M_PI) {
                dteta -= 2 * M_PI;
            } else if (dteta < -M_PI) {
                dteta += 2 * M_PI;
            }

            if (odometry_msg->speed != 0. && odometry_msg->direction != 0.) {
                double speed = odometry_msg->speed * odometry_msg->direction;
                if (std::abs(dteta) < 0.01) {
                    result.x = speed * std::cos(teta) * dt;
                    result.y = speed * std::sin(teta) * dt;
                } else if (current.w_z != 0.) {
                    result.x = speed / current.w_z *
                               (-std::sin(this->prev_dx_dy.teta) + sin(this->prev_dx_dy.teta + current.w_z * dt));
                    result.y = speed / current.w_z *
                               (std::cos(this->prev_dx_dy.teta) - cos(this->prev_dx_dy.teta + current.w_z * dt));
                }
            }
        }

        this->prev_dx_dy = current;
        return result;
    }
};

int main(int argc, char *argv[]) {
    Tracker tracker;
    ros::init(argc, argv, "ballsbot_pose");
    ros::NodeHandle n;

    ros::Subscriber imu_subscriber = n.subscribe("imu", 1, ImuCallback);
    ros::Subscriber odometry_subscriber = n.subscribe("wheel_odometry", 1, OdometryCallback);

    ballsbot_pose::Pose output_msg;
    auto publisher = n.advertise<ballsbot_pose::Pose>("pose", 1);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    Pose prev_pose;

    ros::Rate rate(5);
    while (ros::ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();

        if (current_imu_msg.get() != nullptr && current_odometry_msg.get() != nullptr) {
            auto pose = tracker.GetCurrentPose(current_imu_msg, current_odometry_msg);
            output_msg.header.stamp = current_time;
            output_msg.imu_ts = current_imu_msg->header.stamp;
            output_msg.odometry_ts = current_odometry_msg->header.stamp;
            output_msg.x = pose.x;
            output_msg.y = pose.y;
            output_msg.teta = pose.teta;
            publisher.publish(output_msg);

            //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.teta);

            //first, we'll publish the transform over tf
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = pose.x;
            odom_trans.transform.translation.y = pose.y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = pose.x;
            odom.pose.pose.position.y = pose.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            if (!prev_pose.first_time) {
                auto dt = (current_time - last_time).toSec();
                if (dt) {
                    odom.child_frame_id = "base_link";
                    odom.twist.twist.linear.x = (pose.x - prev_pose.x) / dt;
                    odom.twist.twist.linear.y = (pose.y - prev_pose.y) / dt;
                    odom.twist.twist.angular.z = (pose.teta - prev_pose.teta) / dt;
                }
            }

            //publish the message
            odom_pub.publish(odom);
            prev_pose = pose;
            last_time = current_time;
        }

        rate.sleep();
    }

    return 0;
}
