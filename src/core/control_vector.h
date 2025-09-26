#pragma once

#include "src/core/constants.h"

/**
 * @brief Commanded control input vector for the aircraft (δa, δe, δr, δt).
 */
typedef struct {
    double da;  // Aileron
    double de;  // Elevator
    double dr;  // Rudder
    double dthr[2];  // Throttle
}ControlVector;
