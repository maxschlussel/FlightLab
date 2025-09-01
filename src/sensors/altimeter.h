#pragma once

#include "src/core/state_vector.h"


typedef struct {
    double alt;
} AltimeterSensor;


void readAltimeterSensor(const StateVector* X, AltimeterSensor* altemeterSensor);
