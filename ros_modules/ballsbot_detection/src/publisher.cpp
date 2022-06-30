#include "ros/ros.h"
#include "ballsbot_camera/Image.h"
#include "ballsbot_detection/DetectionsList.h"
#include "cam_detector.h"
#include <string>

const uint8_t kDetectOnCameraIndex = 0;

using ImagePtr = ballsbot_camera::Image::ConstPtr;

ImagePtr current_image_msg;

void CaptureCallback(const ImagePtr &msg) {
    current_image_msg = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ballsbot_detection");
    ros::NodeHandle n;
    ros::Publisher chatter_pub =
        n.advertise<ballsbot_detection::DetectionsList>("cam_detections", 1);

    ros::Subscriber capture_subscriber = n.subscribe(
        std::string("cam_image/") + std::to_string(kDetectOnCameraIndex), 1, CaptureCallback);

    ros::Rate loop_rate(4);

    CamDetector detector;
    detector.StartUp("/home/ballsbot/catkin_ws/src/ballsbot_detection");
    ROS_INFO("detector.StartUp complete");

    int count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        if (current_image_msg.get() != nullptr) {
            ballsbot_detection::DetectionsList msg;

            auto detections =
                detector.Detect(current_image_msg->image, current_image_msg->image_width,
                                current_image_msg->image_height);

            msg.data.resize(detections.size());
            for (size_t i = 0; i < detections.size(); ++i) {
                auto it = detections[i];
                msg.data[i].object_class = it.object_class;
                msg.data[i].confidence = it.confidence;
                msg.data[i].bottom_left.x = it.bottom_left.x;
                msg.data[i].bottom_left.y = it.bottom_left.y;
                msg.data[i].top_right.x = it.top_right.x;
                msg.data[i].top_right.y = it.top_right.y;
                msg.camera_index = kDetectOnCameraIndex;
                // ROS_INFO("%s (%f): (%f, %f), (%f, %f)", msg.data[i].object_class.c_str(),
                //          msg.data[i].confidence, msg.data[i].bottom_left.x,
                //          msg.data[i].bottom_left.y, msg.data[i].top_right.x,
                //          msg.data[i].top_right.y);
            }

            chatter_pub.publish(msg);
        }

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
