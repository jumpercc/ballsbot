#include <cmath>
#include "geometry.h"

LinearCoefficients GetLinearCoefficients(const Point p1, const Point p2) {
    LinearCoefficients result = {p1.y - p2.y, p2.x - p1.x, p1.x * p2.y - p2.x * p1.y};
    return result;
}

double PointToLineDistance(Point p0, LinearCoefficients a_line) {
    return std::abs(a_line.a * p0.x + a_line.b * p0.y + a_line.c) /
           sqrt(a_line.a * a_line.a + a_line.b * a_line.b);
}

double Distance(Point one, Point two) {
    return sqrt((one.x - two.x) * (one.x - two.x) + (one.y - two.y) * (one.y - two.y));
}

LinearCoefficients NormalToLineInPoint(LinearCoefficients a_line, Point a_point) {
    if (a_line.b == 0.) {
        LinearCoefficients result = {0., 1., -a_point.y};
        return result;
    } else {
        LinearCoefficients result = {1., -a_line.a / a_line.b,
                                     -a_point.x + a_line.a / a_line.b * a_point.y};
        return result;
    }
}

std::pair<LinearCoefficients, LinearCoefficients> GetTwoNRadiansLines(double angle,
                                                                      LinearCoefficients a_line,
                                                                      Point a_point) {
    std::pair<LinearCoefficients, LinearCoefficients> result;

    result.first.b = 1.;
    result.second.b = 1.;

    if (a_line.a == 0.) {
        result.first.a = tan(angle);
        result.second.a = tan(-angle);
    } else if (a_line.b == 0.) {
        result.first.a = tan(M_PI / 2. - angle);
        result.second.a = tan(M_PI / 2. + angle);
    } else {
        result.first.a = tan(atan(a_line.a / a_line.b) + angle);
        result.second.a = tan(atan(a_line.a / a_line.b) - angle);
    }

    result.first.c = -result.first.a * a_point.x - result.first.b * a_point.y;
    result.second.c = -result.second.a * a_point.x - result.second.b * a_point.y;

    return result;
}

bool OnOtherSide(LinearCoefficients a_line, Point p1, Point p2) {
    auto second_line = GetLinearCoefficients(p1, p2);
    Point p0;

    if (a_line.b == 0. and second_line.b == 0.) {
        return false;
    } else if (a_line.b == 0.) {
        p0.x = -a_line.c / a_line.a;
        p0.y = -(second_line.a * p0.x + second_line.c) / second_line.b;
    } else if (second_line.b == 0.) {
        p0.x = -second_line.c / second_line.a;
        p0.y = -(a_line.a * p0.x + a_line.c) / a_line.b;
    } else {
        double nom = a_line.c / a_line.b - second_line.c / second_line.b;
        double denom = second_line.a / second_line.b - a_line.a / a_line.b;
        if (denom == 0.) {
            return false;
        } else {
            p0.x = nom / denom;
            p0.y = -(a_line.a * p0.x + a_line.c) / a_line.b;
        }
    }

    if (p1.x > p2.x) {
        if (p1.x <= p0.x || p0.x <= p2.x) {
            return false;
        }
    } else if (p1.x < p2.x) {
        if (p1.x >= p0.x || p0.x >= p2.x) {
            return false;
        }
    }

    if (p1.y > p2.y) {
        if (p1.y <= p0.y || p0.y <= p2.y) {
            return false;
        }
    } else if (p1.y < p2.y) {
        if (p1.y >= p0.y || p0.y >= p2.y) {
            return false;
        }
    }

    return true;
}