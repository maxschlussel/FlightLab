#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/state_vector.h"
#include "src/core/control_vector.h"


void integrateRK4Step(StateVector* X, 
                      ControlVector* U, 
                      AircraftParams* acParams, 
                      const double Xdot[12], 
                      double dt_s);
