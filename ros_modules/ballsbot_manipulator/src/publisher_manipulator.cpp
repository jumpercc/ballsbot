#include <string>
#include <unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "ballsbot_tca9548/EncoderAngle.h"

using ImuPtr = ballsbot_tca9548::EncoderAngle::ConstPtr;

std::unordered_map<std::string, float> angle_by_sensor;

void EncodersCallback(const ImuPtr &msg) {
    angle_by_sensor[msg->sensor_name] = msg->angle;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ballsbot_manipulator");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber encoders_subscriber = n.subscribe("magnetic_encoder", 1, EncodersCallback);
    sensor_msgs::JointState joint_state;

    std::unordered_map<std::string, float> calibration = {  // FIXME read config
        {"m-0", 6.0623f},
        {"m-1", 3.4607f},
        {"m-2", 0.3191f},
        {"m-claw", 2.3562f},  // FIXME m-claw-left + m-claw-right
    };

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();

        if (!angle_by_sensor.empty()) {
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(angle_by_sensor.size());
            joint_state.position.resize(angle_by_sensor.size());
            for(auto it : angle_by_sensor) {
                float value = it.second - calibration[it.first];
                if (value < -M_PI ) {
                    value += 2 * M_PI;
                }
                else if (value > M_PI ) {
                    value -= 2 * M_PI;
                }
                joint_state.name[0] = it.first.c_str();
                joint_state.position[0] = value;
            }

            joint_pub.publish(joint_state);
        }

        loop_rate.sleep();
    }

    return 0;
}