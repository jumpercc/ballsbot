#include <string>
#include <unistd.h>
#include <vector>
#include <cmath>

#include <opencv2/core/matx.hpp>

#include "cam_capture.h"
#include "ros/ros.h"

std::string CamCapture::GstreamerPipeline() {
    return "nvarguscamerasrc sensor-id=" + std::to_string(camera_index_) +
           " ! video/x-raw(memory:NVMM), width=" + std::to_string(kCaptureWidth) +
           ", height=" + std::to_string(kCaptureHeight) +
           ", format=(string)NV12, framerate=(fraction)" + std::to_string(frame_rate_) +
           "/1 ! nvvidconv ! video/x-raw, width=(int)" + std::to_string(display_width_) + ", height=(int)" +
           std::to_string(display_height_) + ", format=(string)BGRx ! videoconvert ! appsink";
}

void CamCapture::OpenVideoStream() {
    std::string pipeline = GstreamerPipeline();
    cap_ = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
    // cap_.set(cv::CAP_PROP_FPS, frame_rate_);
    video_start_ts_ = std::chrono::high_resolution_clock::now();
    frames_seen_ = 0;
}

int CamCapture::GetFramesToSkip() {
    auto now_ts = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> from_video_start = now_ts - video_start_ts_;
    int result = std::floor(double(frame_rate_) * from_video_start.count() / 1000.) - frames_seen_;
    if (result == 1) {
        result = 0;
    }
    return result;
}

std::vector<uint8_t> CamCapture::Capture() {
    if (!cap_.isOpened()) {
        OpenVideoStream();
        if (!cap_.isOpened()) {
            throw std::runtime_error("Failed to open camera");
        }
    }

    if (frames_seen_ != 0) {
        // skip obsolete frames
        auto frames_to_skip = GetFramesToSkip();
        if (frames_to_skip > 0) {
            frames_seen_ += frames_to_skip;
            if (frames_to_skip > frame_rate_) {
                frames_to_skip = frame_rate_;
            }
            ROS_INFO("camera %d: skipping %d frames", camera_index_, frames_to_skip);
            for (int i = 0; i < frames_to_skip - 1; ++i) {
                cap_.grab();
            }
        }
    }

    if (!cap_.read(img_)) {
        throw std::runtime_error("Capture read error");
    }
    ++frames_seen_;

    if (img_.type() != CV_8UC3) {
        throw std::runtime_error("Unexpected capture format (need CV_8UC3)");
    }

    std::vector<uint8_t> result;
    result.resize(3 * display_width_ * display_height_);
    size_t offset = 0;
    for (int r = 0; r < img_.rows; ++r) {
        for (int c = 0; c < img_.cols; ++c) {
            auto &pixel = img_.at<cv::Vec3b>(r, c);
            result[offset++] = pixel[0];
            result[offset++] = pixel[1];
            result[offset++] = pixel[2];
        }
    }
    return result;
}
