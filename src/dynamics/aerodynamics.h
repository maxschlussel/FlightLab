#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computeAerodynamicForces(const StateVector* X, const ControlVector* U, const AircraftParams* acParams, Vector3* F, Vector3* M);

double computeCL(const AircraftParams* acParams, const ControlVector* U, double alpha);

double computeCL_wingbody(const AircraftParams* acParamms, double alpha);

double computeCL_tail(const AircraftParams* acParams, const ControlVector* U, double alpha);

double computeCd(double alpha);

double computeCy(const ControlVector* U, double beta);

Vector3 computeCM(const AircraftParams* acParams, double alpha, double beta, double velocity, Vector3* w_b, Vector3* u_123);

double computeEpsilonDownwash(const AircraftParams* acParams, double alpha);
