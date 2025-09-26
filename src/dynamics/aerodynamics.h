#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computeAerodynamicForces(StateVector* X, ControlVector* U, AircraftParams* acParams, Vector3* F, Vector3* M);

double computeCL(AircraftParams* acParams, ControlVector* U, double alpha);

double computeCL_wingbody(AircraftParams* acParamms, double alpha);

double computeCL_tail(AircraftParams* acParams, ControlVector* U, double alpha);

double computeCd(double alpha);

double computeCy(ControlVector* U, double beta);

Vector3 computeCM(AircraftParams* acParams, double alpha, double beta, double velocity, Vector3* w_b, Vector3* u_123);

double computeEpsilonDownwash(AircraftParams* acParams, double alpha);
