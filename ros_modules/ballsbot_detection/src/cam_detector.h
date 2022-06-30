#pragma once

#include <string>
#include <vector>
#include <chrono>

#include "uffssd.h"

struct DetectionPoint {
    float x, y;
};

struct Detection {
    std::string object_class;
    float confidence;
    DetectionPoint bottom_left, top_right;
};

class CamDetector {
public:
    static const int kDisplayWidth = 300;
    static const int kDisplayHeight = 300;

    ~CamDetector() {
        CleanupDetector();
    }

    void StartUp(std::string net_files_directory);

    std::vector<Detection> Detect(std::vector<uint8_t> image, uint16_t width, uint16_t height);

private:
    std::vector<std::string> classes_names_;
    std::string net_files_directory_;

    void ReadClassesNames();
};