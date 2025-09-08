#pragma once

#include <stdlib.h>

#include "src/core/constants.h"


void deg2rad_array(const double* deg, double* rad, double length);

void rad2deg_array(const double* rad, double* deg, double length);

double clamp(double x, double min, double max);

double randNoise(double stddev) ;
