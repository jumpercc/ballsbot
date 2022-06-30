#pragma once

#include <string>
#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>

class CamCapture {
public:
    static const int kCaptureWidth = 3264;
    static const int kCaptureHeight = 2464;
    static const int kFlipMethod = 0;

    CamCapture(uint8_t camera_index, uint8_t frame_rate, uint16_t width, uint16_t height)
        : camera_index_(camera_index),
          frame_rate_(frame_rate),
          display_width_(width),
          display_height_(height) {
    }

    ~CamCapture() {
        cap_.release();
    }

    std::vector<uint8_t> Capture();

private:
    cv::Mat img_;
    cv::VideoCapture cap_;
    std::chrono::high_resolution_clock::time_point video_start_ts_;
    uint64_t frames_seen_;
    uint8_t camera_index_;
    uint8_t frame_rate_;
    uint16_t display_width_;
    uint16_t display_height_;

    void OpenVideoStream();
    int GetFramesToSkip();
    std::string GstreamerPipeline();
};