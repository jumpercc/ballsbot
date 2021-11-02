#pragma once
#include "grid.h"

using FreeDistances = DirectionsWeights;

FreeDistances GetFreeDistances(const PointCloud& points, CarInfo car_info);
FreeDistances DebugGetFreeDistances(const Grid& grid, double current_ts, CarInfo car_info);