#include "src/actuators/actuators.h"
#include "src/sim/scenarios.h"
#include "src/core/state_vector.h"


/**
* @brief Load the initial conditions of the aircraft state.
*/
StateVector initStateVectorBasicCruise(void){
    StateVector X = {
        .u = 125.0,
        .v = 0.0,
        .w = 0.0,
        .p = 0.0,
        .q = 0.0,
        .r = 0.0,
        .phi    = 0.1,
        .theta  = 0.1,
        .psi    = 0.1,
        .x = 0.0,
        .y = 0.0,
        .z = -1000.0
    };
    return X;
}


/**
* @brief Load the initial conditions of the aircraft control system.
*/
ControlSystemPID initControlSystemPID(void){
    // alt2pitch
    double kp_alt = -0.0012;
    double ki_alt = -0.0001;
    double kd_alt = -0.0;
    double alt_outMin = (-25.0 - 5.0) * deg2rad;
    double alt_outMax = (25.0 - 5.0) * deg2rad;
    
    // pitch2elv
    double kp_pitch = -0.7;
    double ki_pitch = -0.45;
    double kd_pitch = -0.0;
    double pitch_outMin = (-25.0 + 10.2) * deg2rad;
    double pitch_outMax = (10.0 + 10.2) * deg2rad;
    
    // airspeed2throttle
    double kp_airspeed = 0.0015;
    double ki_airspeed = 0.00001;
    double kd_airspeed = 0.0;
    double airspeed_outMin = (0.5 - 4.7) * deg2rad;
    double airspeed_outMax = (10 - 4.7) * deg2rad;
    
    // heading2roll
    double kp_heading = 2.5;
    double ki_heading = 0.0;
    double kd_heading = 0.0;
    double heading_outMin = (-45.0) * deg2rad;
    double heading_outMax = (45.0) * deg2rad;

    // roll2aileronPID
    double kp_roll = -1.1;
    double ki_roll = -0.055;
    double kd_roll = 0.0;
    double roll_outMin = (-25.0) * deg2rad;
    double roll_outMax = (25.0) * deg2rad;

    // control vector
    double da = 0.0;
    double de = -5.0 * deg2rad;
    double dr = 0.0;
    double dthr[2] = {0.0, 0.0};

    ControlSystemPID controlSystemPID = {
        .U_cmd = {.da = da,
                  .de = de,
                  .dr = dr,
                  .dthr[0] = dthr[0],
                  .dthr[1] = dthr[1]},

        .alt2pitchPID = initPID(kp_alt, ki_alt, kd_alt, alt_outMin, alt_outMax),
        .pitch2elvPID = initPID(kp_pitch, ki_pitch, kd_pitch, pitch_outMin, pitch_outMax),
        .airspeed2throtPID = initPID(kp_airspeed, ki_airspeed, kd_airspeed, airspeed_outMin, airspeed_outMax),
        
        .heading2rollPID = initPID(kp_heading, ki_heading, kd_heading, heading_outMin, heading_outMax),
        .roll2aileronPID = initPID(kp_roll, ki_roll, kd_roll, roll_outMin, roll_outMax)
    };

    return controlSystemPID;
}


Actuators initActuators(ControlVector* U_cmd){
    Actuators actuators = {
        .aileronServo  = { .pos = U_cmd->da, 
                           .maxPosLim =  25.0 * deg2rad, 
                           .minPosLim = -25.0 * deg2rad, 
                           .rateLim = 25.0, 
                           .tau = 0.05 },
        .elevatorServo = { .pos = U_cmd->de, 
                           .maxPosLim =  10.0 * deg2rad, 
                           .minPosLim = -25.0 * deg2rad, 
                           .rateLim = 15.0, 
                           .tau = 0.05 },
        .rudderServo   = { .pos = U_cmd->dr, 
                           .maxPosLim =  30.0 * deg2rad, 
                           .minPosLim = -30.0 * deg2rad, 
                           .rateLim = 25.0, 
                           .tau = 0.05 },
        .throttle = {U_cmd->dthr[0], U_cmd->dthr[1]}
    };
    return actuators;
}
