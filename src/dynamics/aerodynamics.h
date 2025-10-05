#pragma once

#include "src/core/aircraft_params.h"
#include "src/actuators/actuators.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


void computeAerodynamicForces(const StateVector* X, const Actuators* actuators, const AircraftParams* acParams, Vector3* F, Vector3* M);

double computeCL(const AircraftParams* acParams, double de, double alpha);

double computeCL_wingbody(const AircraftParams* acParamms, double alpha);

double computeCL_tail(const AircraftParams* acParams, double de, double alpha);

double computeCd(double alpha);

double computeCy(double dr, double beta);

Vector3 computeCM(const AircraftParams* acParams, double alpha, double beta, double velocity, Vector3* w_b, Vector3* u_123);

double computeEpsilonDownwash(const AircraftParams* acParams, double alpha);
