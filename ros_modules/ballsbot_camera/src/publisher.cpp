#include <vector>
#include <string>
#include "ros/ros.h"
#include "ballsbot_camera/Image.h"
#include "cam_capture.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ballsbot_camera");

    uint8_t cameras_count = 2;
    uint8_t frame_rate = 4;
    uint16_t display_width = 640;
    uint16_t display_height = 480;

    if (argc >= 2) {
        cameras_count = atoi(argv[1]);
    }

    ros::NodeHandle n;
    std::vector<ros::Publisher> chatter_pubs;
    ros::Rate loop_rate(frame_rate);

    std::vector<CamCapture> captures;
    ROS_INFO("got %d cameras", cameras_count);
    for (uint8_t camera_index = 0; camera_index < cameras_count; ++camera_index) {
        ROS_INFO("starting camera %d", camera_index);
        chatter_pubs.push_back(n.advertise<ballsbot_camera::Image>(
            std::string("cam_image/") + std::to_string(camera_index), 1));
        captures.emplace_back(camera_index, frame_rate, display_width, display_height);
        ROS_INFO("camera %d started", camera_index);
    }
    ROS_INFO("cameras started");

    std::vector<std::vector<uint8_t>> images;
    images.resize(cameras_count);
    while (ros::ok()) {
        for (uint8_t camera_index = 0; camera_index < cameras_count; ++camera_index) {
            images[camera_index] = captures[camera_index].Capture();
            ballsbot_camera::Image msg;
            msg.header.stamp = ros::Time::now();
            msg.camera_index = camera_index;
            msg.image_width = display_width;
            msg.image_height = display_height;
            msg.image = images[camera_index];
            chatter_pubs[camera_index].publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
