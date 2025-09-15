#pragma once

#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/sensors/sensors.h"


void computeFlightControls(Sensors* sensors, ControlVector* U,AircraftParams* acParams);
