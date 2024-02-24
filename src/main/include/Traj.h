#pragma once

#include <math.h>

#include "Constants.h"

double getAngle(const double velocity, const double distance, const double height);

bool isInRange(const double velocity, const double distance, const double angle);

double getDistance(const double ll_angle, const double ll_height, const double target_height);