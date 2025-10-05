#pragma once

/**
 * @brief Represents a single servo actuator.
 */
typedef struct {
    double pos;         // Actuator position (delta from predefined 0 point)
    double maxPosLim;   // Maximum absolute deflection
    double minPosLim;   // Minimum absoute deflection
    double rateLim;     // Max deflection rate
    double tau;         // Time constant [s] for first-order lag
}ServoActuator;


void actuateServo(ServoActuator* servo, double U_cmd, double dt);
