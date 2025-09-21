#pragma once

#include "src/core/control_vector.h"
#include "src/actuators/servo.h"

typedef struct {
    ServoActuator aileronServo;
    ServoActuator elevatorServo;
    ServoActuator rudderServo;
    double throttle[2];
} Actuators;


void driveActuators(ControlVector* U_cmd, Actuators* actuators, double dt);
