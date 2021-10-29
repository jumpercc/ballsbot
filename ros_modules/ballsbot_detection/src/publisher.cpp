#include "ros/ros.h"
#include "ballsbot_detection/DetectionsList.h"
#include "cam_detector.h"
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ballsbot_detection");
    ros::NodeHandle n;
    ros::Publisher chatter_pub =
        n.advertise<ballsbot_detection::DetectionsList>("cam_detections", 1);
    ros::Rate loop_rate(10);

    CamDetector detector;
    detector.StartUp("/home/ballsbot/catkin_ws/src/ballsbot_detection");

    int count = 0;
    while (ros::ok()) {
        ballsbot_detection::DetectionsList msg;

        auto detections = detector.Detect();

        msg.data.resize(detections.size());
        for (size_t i = 0; i < detections.size(); ++i) {
            auto it = detections[i];
            msg.data[i].object_class = it.object_class;
            msg.data[i].confidence = it.confidence;
            msg.data[i].bottom_left.x = it.bottom_left.x;
            msg.data[i].bottom_left.y = it.bottom_left.y;
            msg.data[i].top_right.x = it.top_right.x;
            msg.data[i].top_right.y = it.top_right.y;
            // ROS_INFO("%s (%f): (%f, %f), (%f, %f)", msg.data[i].object_class.c_str(),
            //          msg.data[i].confidence, msg.data[i].bottom_left.x, msg.data[i].bottom_left.y,
            //          msg.data[i].top_right.x, msg.data[i].top_right.y);
        }

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
