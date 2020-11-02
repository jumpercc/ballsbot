#pragma once

#include <vector>
#include "common.h"

struct CloudTransformation {
    double dx, dy, dteta;

    CloudTransformation(double dx = 0., double dy = 0., double dteta = 0.)
        : dx(dx), dy(dy), dteta(dteta) {
    }
};

using PointCloud = std::vector<Point>;

PointCloud ApplyTransformation(const PointCloud& source_cloud, CloudTransformation transformation);
