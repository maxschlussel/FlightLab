#pragma once

#include "src/core/state_vector.h"

/**
 * Basic representation of an altimeter sensor.
 */
typedef struct {
    double alt;
} AltimeterSensor;


void readAltimeterSensor(const StateVector* X, AltimeterSensor* altemeterSensor);
