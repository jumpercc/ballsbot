#include <vector>
#include <cmath>
#include "point_cloud.h"

PointCloud ApplyTransformation(const PointCloud& source_cloud, CloudTransformation transformation) {
    PointCloud result = source_cloud;

    double rotate11 = cos(transformation.dteta);
    double rotate12 = -sin(transformation.dteta);
    double rotate21 = sin(transformation.dteta);
    double rotate22 = cos(transformation.dteta);

    double move11 = transformation.dx;
    double move21 = transformation.dy;

    for (size_t i = 0; i < source_cloud.size(); ++i) {
        auto a_point = source_cloud[i];
        result[i].x = rotate11 * a_point.x + rotate12 * a_point.y + move11;
        result[i].y = rotate21 * a_point.x + rotate22 * a_point.y + move21;
    }

    return result;
}
