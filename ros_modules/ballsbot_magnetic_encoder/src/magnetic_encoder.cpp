#include "magnetic_encoder_driver.h"
#include "ros/ros.h"
#include "ballsbot_magnetic_encoder/EncoderAngle.h"
#include <string>
#include <sstream>

int main(int argc, char **argv) {
    unsigned char bus = 0;
    std::string encoder_name("default");
    std::string topic_name("magnetic_encoder_");
    topic_name += encoder_name;

    if (argc >= 2) {
        bus = atoi(argv[1]);
    }

    ros::init(argc, argv, "ballsbot_magnetic_encoder");

    ros::NodeHandle n;
    ros::Publisher chatter_pub =
        n.advertise<ballsbot_magnetic_encoder::EncoderAngle>(topic_name, 1);
    ros::Rate loop_rate(10);

    AMS_AS5600 a_sensor(bus);
    a_sensor.OpenSensor();
    a_sensor.SetClockWise(false);

    while (ros::ok()) {
        ballsbot_magnetic_encoder::EncoderAngle msg;

        msg.angle = a_sensor.GetAngle(U_RAD, true);
        msg.encoder_name = encoder_name;

        ROS_INFO("%s: %0.4f", msg.encoder_name.c_str(), msg.angle);
        ROS_INFO("-");

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    a_sensor.CloseSensor();
}