#pragma once

#include <stdbool.h>
#include <stdlib.h>

#include "src/core/constants.h"


void deg2rad_array(const double* deg, double* rad, double length);

void rad2deg_array(const double* rad, double* deg, double length);

double clamp(double x, double min, double max);

bool isEqual(double val1, double val2, double eps);

double randNoise(double stddev) ;
