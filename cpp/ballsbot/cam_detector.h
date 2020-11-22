#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include "uffssd.h"

struct DetectionPoint {
    float x, y;
};

struct Detection {
    std::string object_class;
    float confidence;
    DetectionPoint top_left, bottom_right;
};

class CamDetector {
public:
    static const int kCaptureWidth = 1280;  // 3264;
    static const int kCaptureHeight = 720;  // 2464;
    static const int kDisplayWidth = 300;
    static const int kDisplayHeight = 300;
    static const int kFramerate = 1;
    static const int kFlipMethod = 0;

    CamDetector() {
        std::string pipeline = GstreamerPipeline();
        cap_ = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
    }

    ~CamDetector() {
        cap_.release();
        CleanupDetector();
    }

    void StartUp(std::string net_files_directory);

    std::vector<Detection> Detect();

private:
    cv::Mat img_;
    uint8_t raw_bgr_[kDisplayWidth * kDisplayHeight * 3];
    cv::VideoCapture cap_;
    std::vector<std::string> classes_names_;
    std::string net_files_directory_;

    void ReadClassesNames();
    std::string GstreamerPipeline();
};