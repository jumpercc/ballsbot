#pragma once

#include <vector>
#include <string>

void StartupDetector(std::string net_files_directory);

/*
 * [class_number, confidence, x0, y0, x1, y1]
 * x, y in [0-1]
 */
std::vector<std::vector<float>> DetectObjects(uint8_t *bgr_data);

void CleanupDetector();
