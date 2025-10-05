#pragma once

#include "src/math/vector.h"

#define EPS 1e-8

// Global constants in metric
extern const double pi;
extern const double twoPi;
extern const double deg2rad;
extern const double rad2deg;

extern const double g;  // Gravity [m/s^2]

extern const Vector3 B_ned;  // Earth magnetic field in NED

extern const double rho;  // Air density at MSL [kg/m^3]
