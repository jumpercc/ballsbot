#include "free_distances.h"
#include "geometry.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>

const double kFearDistance = 0.05;
const double kMinInnerOffset = 0.03;
const double kStopDistance = 0.35;

struct BodyPosition {
    BodyPosition(CarInfo car_info) {
        x_front = car_info.to_car_center + car_info.car_length / 2.;
        x_rear = car_info.to_car_center - car_info.car_length / 2.;
        y_left = car_info.car_width / 2.;
        y_right = -car_info.car_width / 2.;

        front_offset = std::max(std::abs(x_front / 3.), kMinInnerOffset);
        rear_offset = std::max(std::abs(x_rear / 3.), kMinInnerOffset);
        left_offset = std::max(std::abs(y_left / 3.), kMinInnerOffset);
        right_offset = left_offset;

        x_rear_with_offset = x_rear + rear_offset;
        x_front_with_offset = x_front - front_offset;
        y_left_with_offset = y_left - left_offset;
        y_right_with_offset = y_right + right_offset;
    }

    double x_rear, x_front, y_right, y_left;
    double rear_offset, front_offset, right_offset, left_offset;
    double x_rear_with_offset, x_front_with_offset, y_right_with_offset, y_left_with_offset;
};

PointCloud FilterNearbyPoints(const PointCloud& nearby_points, CarInfo car_info,
                              BodyPosition body_position, double check_radius) {
    Point left_column_center = {car_info.to_pivot_center, car_info.turn_radius};
    Point right_column_center = {car_info.to_pivot_center, -car_info.turn_radius};
    double column_radius = car_info.turn_radius - car_info.car_width / 2. - kFearDistance;

    PointCloud result;

    std::copy_if(nearby_points.begin(), nearby_points.end(), std::back_inserter(result),
                 [left_column_center, right_column_center, column_radius, &body_position,
                  check_radius](Point point) {
                     if (std::sqrt(point.x * point.x + point.y * point.y) > check_radius) {
                         return false;
                     }

                     if (point.y > 0.) {
                         if (Distance(left_column_center, point) < column_radius) {
                             return false;
                         } else if (point.y > left_column_center.y &&
                                    left_column_center.x - column_radius < point.x &&
                                    point.x < left_column_center.x + column_radius) {
                             return false;
                         }
                     } else if (point.y < 0.) {
                         if (Distance(right_column_center, point) < column_radius) {
                             return false;
                         } else if (point.y < right_column_center.y &&
                                    right_column_center.x - column_radius < point.x &&
                                    point.x < right_column_center.x + column_radius) {
                             return false;
                         }
                     }

                     if (body_position.x_rear_with_offset <= point.x &&
                         point.x <= body_position.x_front_with_offset &&
                         body_position.y_right_with_offset <= point.y &&
                         point.y <= body_position.y_left_with_offset) {
                         return false;
                     }
                     return true;
                 });

    return result;
}

using FilterType = PointCloud(const PointCloud&, CarInfo);

double CanMoveSomeForward(const PointCloud& nearby_points, CarInfo car_info,
                          BodyPosition body_position, double check_radius, FilterType filter) {
    PointCloud filtered_points = filter(nearby_points, car_info);
    Point front_center = {body_position.x_front, 0.};

    double result = check_radius;
    double distance;
    for (auto point : filtered_points) {
        distance = Distance(point, front_center);
        if (distance < result) {
            if (distance <= kStopDistance) {
                return 0.;
            } else {
                result = distance;
            }
        }
    }
    if (result < 0.) {
        throw std::runtime_error("CanMoveSomeForward: result < 0");
    }
    return result;
}

double CanMoveSomeBackward(const PointCloud& nearby_points, CarInfo car_info,
                           BodyPosition body_position, double check_radius, FilterType filter) {
    PointCloud filtered_points = filter(nearby_points, car_info);
    Point rear_center = {body_position.x_rear, 0.};

    double result = check_radius;
    double distance;
    for (auto point : filtered_points) {
        distance = Distance(point, rear_center);
        if (distance < result) {
            if (distance <= kStopDistance) {
                return 0.;
            } else {
                result = distance;
            }
        }
    }
    if (result < 0.) {
        throw std::runtime_error("CanMoveSomeBackward: result < 0");
    }
    return result;
}

bool Between(double value, double left, double right) {
    return left < value && value < right;
}

PointCloud FilterABitRightPoints(const PointCloud& nearby_points, CarInfo car_info) {
    double check_radius = 2. * car_info.turn_radius;
    Point right_center = {car_info.to_pivot_center, -check_radius};
    double center_radius = check_radius;
    double outer_radius = center_radius + car_info.car_width / 2. + kFearDistance;
    double inner_radius = center_radius - car_info.car_width / 2. - kFearDistance;

    PointCloud result;

    std::copy_if(nearby_points.begin(), nearby_points.end(), std::back_inserter(result),
                 [right_center, inner_radius, outer_radius](Point point) {
                     return Between(Distance(right_center, point), inner_radius, outer_radius);
                 });

    return result;
}

PointCloud FilterABitLeftPoints(const PointCloud& nearby_points, CarInfo car_info) {
    double check_radius = 2. * car_info.turn_radius;
    Point right_center = {car_info.to_pivot_center, check_radius};
    double center_radius = check_radius;
    double outer_radius = center_radius + car_info.car_width / 2. + kFearDistance;
    double inner_radius = center_radius - car_info.car_width / 2. - kFearDistance;

    PointCloud result;

    std::copy_if(nearby_points.begin(), nearby_points.end(), std::back_inserter(result),
                 [right_center, inner_radius, outer_radius](Point point) {
                     return Between(Distance(right_center, point), inner_radius, outer_radius);
                 });

    return result;
}

PointCloud FilterMaxRightPoints(const PointCloud& nearby_points, CarInfo car_info) {
    Point right_column_center = {car_info.to_pivot_center, -car_info.turn_radius};
    double outer_radius = car_info.turn_radius + car_info.car_width / 2. + kFearDistance;

    PointCloud result;

    std::copy_if(nearby_points.begin(), nearby_points.end(), std::back_inserter(result),
                 [right_column_center, outer_radius](Point point) {
                     return Distance(right_column_center, point) < outer_radius;
                 });

    return result;
}

PointCloud FilterMaxLeftPoints(const PointCloud& nearby_points, CarInfo car_info) {
    Point left_column_center = {car_info.to_pivot_center, car_info.turn_radius};
    double outer_radius = car_info.turn_radius + car_info.car_width / 2. + kFearDistance;

    PointCloud result;

    std::copy_if(nearby_points.begin(), nearby_points.end(), std::back_inserter(result),
                 [left_column_center, outer_radius](Point point) {
                     return Distance(left_column_center, point) < outer_radius;
                 });

    return result;
}

double CanMoveStraightForward(const PointCloud& nearby_points, BodyPosition body_position,
                              double check_radius) {
    double min_y = body_position.y_right - kFearDistance;
    double max_y = body_position.y_left + kFearDistance;
    double min_x = body_position.x_front_with_offset;
    double max_x = body_position.x_front + check_radius;
    double stop_distance = kStopDistance + body_position.front_offset;

    double result = check_radius;
    double distance;
    for (auto point : nearby_points) {
        if (min_x < point.x && point.x < max_x && min_y <= point.y && point.y <= max_y) {
            distance = std::abs(point.x - min_x);
            if (distance < result) {
                if (distance < stop_distance) {
                    return 0.;
                } else {
                    result = distance;
                }
            }
        }
    }
    if (result < 0.) {
        throw std::runtime_error("CanMoveStraightForward: result < 0");
    }
    return result;
}

double CanMoveABitLeftForward(const PointCloud& nearby_points, CarInfo car_info,
                              BodyPosition body_position, double check_radius) {
    return CanMoveSomeForward(nearby_points, car_info, body_position, check_radius,
                              FilterABitLeftPoints);
}

double CanMoveMaxLeftForward(const PointCloud& nearby_points, CarInfo car_info,
                             BodyPosition body_position, double check_radius) {
    return CanMoveSomeForward(nearby_points, car_info, body_position, check_radius,
                              FilterMaxLeftPoints);
}

double CanMoveMaxLeftBackward(const PointCloud& nearby_points, CarInfo car_info,
                              BodyPosition body_position, double check_radius) {
    return CanMoveSomeBackward(nearby_points, car_info, body_position, check_radius,
                               FilterMaxLeftPoints);
}

double CanMoveABitLeftBackward(const PointCloud& nearby_points, CarInfo car_info,
                               BodyPosition body_position, double check_radius) {
    return CanMoveSomeBackward(nearby_points, car_info, body_position, check_radius,
                               FilterABitLeftPoints);
}

double CanMoveStraightBackward(const PointCloud& nearby_points, BodyPosition body_position,
                               double check_radius) {
    double min_y = body_position.y_right - kFearDistance;
    double max_y = body_position.y_left + kFearDistance;
    double max_x = body_position.x_rear_with_offset;
    double min_x = body_position.x_rear - check_radius;
    double stop_distance = kStopDistance + body_position.rear_offset;

    double result = check_radius;
    double distance;
    for (auto point : nearby_points) {
        if (min_x < point.x && point.x < max_x && min_y <= point.y && point.y <= max_y) {
            distance = std::abs(max_x - point.x);
            if (distance < result) {
                if (distance < stop_distance) {
                    return 0.;
                } else {
                    result = distance;
                }
            }
        }
    }
    if (result < 0.) {
        throw std::runtime_error("CanMoveStraightBackward: result < 0");
    }
    return result;
}
double CanMoveABitRightBackward(const PointCloud& nearby_points, CarInfo car_info,
                                BodyPosition body_position, double check_radius) {
    return CanMoveSomeBackward(nearby_points, car_info, body_position, check_radius,
                               FilterABitRightPoints);
}

double CanMoveMaxRightBackward(const PointCloud& nearby_points, CarInfo car_info,
                               BodyPosition body_position, double check_radius) {
    return CanMoveSomeBackward(nearby_points, car_info, body_position, check_radius,
                               FilterMaxRightPoints);
}

double CanMoveMaxRightForward(const PointCloud& nearby_points, CarInfo car_info,
                              BodyPosition body_position, double check_radius) {
    return CanMoveSomeForward(nearby_points, car_info, body_position, check_radius,
                              FilterMaxRightPoints);
}

double CanMoveABitRightForward(const PointCloud& nearby_points, CarInfo car_info,
                               BodyPosition body_position, double check_radius) {
    return CanMoveSomeForward(nearby_points, car_info, body_position, check_radius,
                              FilterABitRightPoints);
}

FreeDistances GetFreeDistances(const PointCloud& points, CarInfo car_info) {
    double check_radius = 2. * car_info.turn_radius;
    BodyPosition body_position = {car_info};

    PointCloud nearby_points = FilterNearbyPoints(points, car_info, body_position, check_radius);

    // TODO optimize to call each Filter*Points once
    FreeDistances result = {
        {{kStraightDirection, kForwardDirection},
         CanMoveStraightForward(nearby_points, body_position, check_radius)},
        {{kABitLeftDirection, kForwardDirection},
         CanMoveABitLeftForward(nearby_points, car_info, body_position, check_radius)},
        {{kMaxLeftDirection, kForwardDirection},
         CanMoveMaxLeftForward(nearby_points, car_info, body_position, check_radius)},
        {{kMaxLeftDirection, kBackwardDirection},
         CanMoveMaxLeftBackward(nearby_points, car_info, body_position, check_radius)},
        {{kABitLeftDirection, kBackwardDirection},
         CanMoveABitLeftBackward(nearby_points, car_info, body_position, check_radius)},
        {{kStraightDirection, kBackwardDirection},
         CanMoveStraightBackward(nearby_points, body_position, check_radius)},
        {{kABitRightDirection, kBackwardDirection},
         CanMoveABitRightBackward(nearby_points, car_info, body_position, check_radius)},
        {{kMaxRightDirection, kBackwardDirection},
         CanMoveMaxRightBackward(nearby_points, car_info, body_position, check_radius)},
        {{kMaxRightDirection, kForwardDirection},
         CanMoveMaxRightForward(nearby_points, car_info, body_position, check_radius)},
        {{kABitRightDirection, kForwardDirection},
         CanMoveABitRightForward(nearby_points, car_info, body_position, check_radius)},
    };
    return result;
}

FreeDistances DebugGetFreeDistances(const Grid& grid, double current_ts, CarInfo car_info) {
    return GetFreeDistances(grid.GetSparsePointCloud(current_ts, 0., false), car_info);
}