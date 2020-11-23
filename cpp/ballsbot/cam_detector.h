#pragma once

#include <string>
#include <vector>
#include <chrono>

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
    static const int kCaptureWidth = 3264;   // 1280;
    static const int kCaptureHeight = 2464;  // 720;
    static const int kDisplayWidth = 300;
    static const int kDisplayHeight = 300;
    static const int kFramerate = 2;
    static const int kFlipMethod = 0;

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
    std::chrono::high_resolution_clock::time_point video_start_ts_;
    uint64_t frames_seen_;

    void OpenVideoStream();
    int GetFramesToSkip();
    void ReadClassesNames();
    std::string GstreamerPipeline();
};