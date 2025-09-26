#pragma once

#include "src/core/control_vector.h"
#include "src/actuators/servo.h"

/**
 * @brief Represents the physical actuators of the aircraft.
 *
 * This struct contains the the various physical actuators of the vehicle, 
 * including the servos for control surfaces and the throttles for
 * propulsion.
 */
typedef struct {
    ServoActuator aileronServo;
    ServoActuator elevatorServo;
    ServoActuator rudderServo;
    double throttle[2];
} Actuators;


void driveActuators(ControlVector* U_cmd, Actuators* actuators, double dt);
