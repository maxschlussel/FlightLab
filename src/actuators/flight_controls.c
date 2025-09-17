#include "src/actuators/flight_controls.h"
#include "src/core/constants.h"

#include "src/io/logger.h"


FlightControls initFlightControls(ControlVector* U_cmd){
    FlightControls flightControls = {
        .aileronServo  = { .pos = U_cmd->da, 
                           .maxPosLim =  25 * deg2rad, 
                           .minPosLim = -25 * deg2rad, 
                           .rateLim = 25.0, 
                           .tau = 0.05 },
        .elevatorServo = { .pos = U_cmd->de, 
                           .maxPosLim =  10 * deg2rad, 
                           .minPosLim = -25 * deg2rad, 
                           .rateLim = 15.0, 
                           .tau = 0.05 },
        .rudderServo   = { .pos = U_cmd->dr, 
                           .maxPosLim =  30 * deg2rad, 
                           .minPosLim = -30 * deg2rad, 
                           .rateLim = 25.0, 
                           .tau = 0.05 },
        .throttle = {U_cmd->dt[0], U_cmd->dt[1]}
    };
    return flightControls;
}


void actuateFlightControls(ControlVector* U_cmd, FlightControls* flightControls, double dt){
    actuateServo(&(flightControls->aileronServo), U_cmd->da, dt);
    actuateServo(&(flightControls->elevatorServo), U_cmd->de, dt);
    actuateServo(&(flightControls->rudderServo), U_cmd->dr, dt);
    flightControls->throttle[0] = U_cmd->dt[0];
    flightControls->throttle[1] = U_cmd->dt[1];

    logger.data[LOG_SRVO_AILERON_POS]   = flightControls->aileronServo.pos;
    logger.data[LOG_SRVO_ELEVATOR_POS]  = flightControls->elevatorServo.pos;
    logger.data[LOG_SRVO_RUDDER_POS]    = flightControls->rudderServo.pos;
    logger.data[LOG_SRVO_THROTTLE1_VAL] = flightControls->throttle[0];
    logger.data[LOG_SRVO_THROTTLE2_VAL] = flightControls->throttle[1];
}


// double throttleLim[2]   = {0.5 * deg2rad, 10 * deg2rad};
// double dThrottleRateLim = 1.6 * deg2rad;  // rate limit for change in throttle [units/sec]
