#pragma once

#include "src/core/constants.h"

/**
 * @brief Control input vector for the aircraft (δa, δe, δr, δt).
 */
typedef struct {
    double da;  // Aileron
    double de;  // Elevator
    double dr;  // Rudder
    double dt[2];  // Throttle
}ControlVector;
