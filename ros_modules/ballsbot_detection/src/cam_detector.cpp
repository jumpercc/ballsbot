#include <string>
#include <unistd.h>
#include <fstream>  // needed by GetClassesNames
#include <stdexcept>
#include <cmath>

#include <opencv2/core/matx.hpp>

#include "uffssd.h"
#include "cam_detector.h"

std::string CamDetector::GstreamerPipeline() {
    return "nvarguscamerasrc sensor-id=" + std::to_string(kCameraIndex) +
           " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(kCaptureWidth) +
           ", height=(int)" + std::to_string(kCaptureHeight) +
           ", format=(string)NV12, framerate=(fraction)" + std::to_string(kFramerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(kFlipMethod) +
           " ! video/x-raw, width=(int)" + std::to_string(kDisplayWidth) + ", height=(int)" +
           std::to_string(kDisplayHeight) + ", format=(string)BGRx ! videoconvert ! appsink";
}

void CamDetector::ReadClassesNames() {
    classes_names_.resize(91);
    std::ifstream label_file(net_files_directory_ + "/ssd_coco_labels.txt");
    std::string line;
    int id = 0;
    while (getline(label_file, line)) {
        classes_names_[id++] = line;
    }
}

void CamDetector::StartUp(std::string net_files_directory) {
    net_files_directory_ = net_files_directory;
    ReadClassesNames();
    StartupDetector(net_files_directory);
}

void CamDetector::OpenVideoStream() {
    std::string pipeline = GstreamerPipeline();
    cap_ = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
    cap_.set(cv::CAP_PROP_FPS, kFramerate);
    video_start_ts_ = std::chrono::high_resolution_clock::now();
    frames_seen_ = 0;
}

int CamDetector::GetFramesToSkip() {
    auto now_ts = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> from_video_start = now_ts - video_start_ts_;
    return std::floor(double(kFramerate) * from_video_start.count() / 1000.) - frames_seen_;
}

std::vector<Detection> CamDetector::Detect() {
    if (!cap_.isOpened()) {
        OpenVideoStream();
        if (!cap_.isOpened()) {
            throw std::runtime_error("Failed to open camera");
        }
    }

    if (frames_seen_ != 0) {
        // skip obsolete frames
        auto frames_to_skip = GetFramesToSkip();
        if (frames_to_skip > kFramerate) {
            frames_to_skip = kFramerate;
        }
        for (int i = 0; i < frames_to_skip - 1; ++i) {
            cap_.grab();
            ++frames_seen_;
        }
    }

    if (!cap_.read(img_)) {
        throw std::runtime_error("Capture read error");
    }
    int to_add = GetFramesToSkip();
    if (to_add < 0) {
        to_add = 0;
    }
    frames_seen_ += static_cast<uint64_t>(to_add);

    if (img_.type() != CV_8UC3) {
        throw std::runtime_error("Unexpected capture format (need CV_8UC3)");
    }

    size_t offset = 0;
    for (int r = 0; r < img_.rows; ++r) {
        for (int c = 0; c < img_.cols; ++c) {
            auto &pixel = img_.at<cv::Vec3b>(r, c);
            raw_bgr_[offset++] = pixel[2];
            raw_bgr_[offset++] = pixel[1];
            raw_bgr_[offset++] = pixel[0];
        }
    }

    std::vector<Detection> result;
    auto detections = DetectObjects(&raw_bgr_[0]);
    for (auto it : detections) {
        Detection det = {
            classes_names_[int(it[0])], it[1], {it[2], 1.f - it[5]}, {it[4], 1.f - it[3]}};
        result.push_back(det);
    }
    return result;
}
