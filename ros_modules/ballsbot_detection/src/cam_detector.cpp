#include <string>
#include <unistd.h>
#include <fstream>  // needed by GetClassesNames

#include <opencv2/imgproc.hpp>
#include <opencv2/core/matx.hpp>

#include "uffssd.h"
#include "cam_detector.h"

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

std::vector<Detection> CamDetector::Detect(std::vector<uint8_t> image, uint16_t width,
                                           uint16_t height) {
    cv::Mat big_cv_image = cv::Mat(height, width, CV_8UC3, image.data());
    cv::Mat small_cv_image(kDisplayHeight, kDisplayWidth, CV_8UC3);
    cv::resize(big_cv_image, small_cv_image, {kDisplayWidth, kDisplayHeight}, 0, 0, cv::INTER_AREA);

    uint8_t raw_bgr[kDisplayWidth * kDisplayHeight * 3];
    size_t offset = 0;
    for (int r = 0; r < small_cv_image.rows; ++r) {
        for (int c = 0; c < small_cv_image.cols; ++c) {
            auto &pixel = small_cv_image.at<cv::Vec3b>(r, c);
            raw_bgr[offset++] = pixel[2];
            raw_bgr[offset++] = pixel[1];
            raw_bgr[offset++] = pixel[0];
        }
    }

    std::vector<Detection> result;
    auto detections = DetectObjects(&raw_bgr[0]);
    for (auto it : detections) {
        Detection det = {
            classes_names_[int(it[0])], it[1], {it[2], 1.f - it[5]}, {it[4], 1.f - it[3]}};
        result.push_back(det);
    }
    return result;
}
