#pragma once

#include <utility>
#include "common.h"

struct LinearCoefficients {
    double a, b, c;
};

LinearCoefficients GetLinearCoefficients(const Point p1, const Point p2);

double PointToLineDistance(Point p0, LinearCoefficients abc);

double Distance(Point one, Point two);

LinearCoefficients NormalToLineInPoint(LinearCoefficients a_line, Point a_point);

std::pair<LinearCoefficients, LinearCoefficients> GetTwoNRadiansLines(double angle,
                                                                      LinearCoefficients a_line,
                                                                      Point a_point);

bool OnOtherSide(LinearCoefficients a_line, Point p1, Point p2);
