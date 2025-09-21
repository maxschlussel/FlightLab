#include "src/controllers/PID/PID.h"
#include "src/core/constants.h"
#include "src/math/utils.h"

#include "src/io/logger.h"


PID initPID(double kp, double ki, double kd, double outMin, double outMax){
    PID pid = {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .integral = 0.0,
        .prevErr = 0.0,
        .outMin = outMin,
        .outMax = outMax
    };
    return pid;
}


float computePID(PID* pid, double err, double dt_s){
    // Compute integral
    double newIntegral = pid->integral + err * dt_s;

    // Compute derivative
    double deriv = (err - pid->prevErr) / dt_s;
    
    // Compute total PID output
    double output = pid->kp * err 
                  + pid->ki * pid->integral 
                  + pid->kd * deriv;
    
    double saturatedOutput = clamp(output, pid->outMin, pid->outMax);

    // Anti windup - only update integral if not saturated
    bool isSaturatted = !isEqual(saturatedOutput, output, EPS);
    if ( !isSaturatted ) {
        pid->integral = newIntegral;
    }
    
    pid->prevErr = err;
    return saturatedOutput;
}
