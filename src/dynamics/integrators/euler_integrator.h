#pragma once

#include "src/core/state_vector.h"

void integrateEulerStep(StateVector* X, const double* Xdot, double dt_s);