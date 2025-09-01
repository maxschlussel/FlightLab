#pragma once

#include "src/core/control_vector.h"

typedef struct {
    double pos;         // Actuator position (delta from predefined 0 point)
    double maxPosLim;   // Maximum absolute deflection
    double minPosLim;   // Minimum absoute deflection
    double rateLim;     // Max deflection rate
    double tau;         // Time constant [s] for first-order lag (servo speed)~
}ServoActuator;


void actuateServo(ServoActuator* servo, double U_cmd, double dt);
