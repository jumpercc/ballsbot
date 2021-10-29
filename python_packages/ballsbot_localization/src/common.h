#pragma once

struct Point {
    Point(double x_value, double y_value) : x(x_value), y(y_value) {
    }
    Point() : x(0.), y(0.) {
    }
    double x, y;
};
