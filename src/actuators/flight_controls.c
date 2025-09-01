#include "src/actuators/flight_controls.h"


FlightControls initFlightControls(ControlVector* U){
    FlightControls flightControls = {
        .aileronServo = {0.0},
        .elevatorServo = {0.0},
        .rudderServo = {0.0},
        .throttle = {0.0}
    };
    return flightControls;
}


void actuateFlightControls(ControlVector* U, FlightControls* flightControls, double dt){
    actuateServo(&(flightControls->aileronServo), U->da, dt);
    actuateServo(&(flightControls->elevatorServo), U->de, dt);
    actuateServo(&(flightControls->rudderServo), U->dr, dt);
    flightControls->throttle[0] = U->dt[0];
    flightControls->throttle[1] = U->dt[1];
}

// double aileronLim[2]    = {-25 * deg2rad, 25 * deg2rad};  //rate lm 25 -25

// double elevatorLim[2]   = {-25 * deg2rad, 10 * deg2rad};  //rate lim 15 -15

// double rudderLim[2]     = {-30 * deg2rad, 30 * deg2rad}; //rate lm 25 -25

// double throttleLim[2]   = {0.5 * deg2rad, 10 * deg2rad};
// double dThrottleRateLim = 1.6 * deg2rad;  // rate limit for change in throttle [units/sec]
