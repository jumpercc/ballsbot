#include <string>
#include <unistd.h>
#include <fstream>  // needed by GetClassesNames
#include <stdexcept>

#include <opencv2/core/matx.hpp>

#include "uffssd.h"
#include "cam_detector.h"

std::string CamDetector::GstreamerPipeline() {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" +
           std::to_string(kCaptureWidth) + ", height=(int)" + std::to_string(kCaptureHeight) +
           ", format=(string)NV12, framerate=(fraction)" + std::to_string(kFramerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(kFlipMethod) +
           " ! video/x-raw, width=(int)" + std::to_string(kDisplayWidth) + ", height=(int)" +
           std::to_string(kDisplayHeight) +
           ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
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
    if (!cap_.isOpened()) {
        throw std::runtime_error("Failed to open camera");
    }
    ReadClassesNames();
    StartupDetector(net_files_directory);
}

std::vector<Detection> CamDetector::Detect() {
    if (!cap_.read(img_)) {
        throw std::runtime_error("Capture read error");
    }

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
        Detection det = {classes_names_[int(it[0])], it[1], {it[2], it[3]}, {it[4], it[5]}};
        result.push_back(det);
    }
    return result;
}
