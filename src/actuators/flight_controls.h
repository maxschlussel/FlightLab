#pragma once

#include "src/core/control_vector.h"
#include "src/actuators/servo.h"

typedef struct {
    ServoActuator aileronServo;
    ServoActuator elevatorServo;
    ServoActuator rudderServo;
    double throttle[2];
} FlightControls;

FlightControls initFlightControls(ControlVector* U);

void actuateFlightControls(ControlVector* U, FlightControls* flightControls, double dt);
