#include "src/controllers/PID/PID.h"
#include "src/core/constants.h"
#include "src/math/utils.h"

#include "src/io/logger.h"


/**
 * @brief Initializes a PID controller struct.
 *
 * @param[in] kp       The proportional gain.
 * @param[in] ki       The integral gain.
 * @param[in] kd       The derivative gain.
 * @param[in] outMin   The minimum output value.
 * @param[in] outMax   The maximum output value.
 * @return PID         An initialized PID struct.
 */
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


/**
 * @brief Computes the output of a PID controller.
 *
 * This function calculates the PID controller output based on the current error,
 * updates the integral and previous error for the next time step, and applies
 * anti-windup to prevent integral buildup when the output is saturated.
 *
 * @param[in,out] pid   A pointer to the PID struct. The integral and previous
 *                      error values within this struct will be updated.
 * @param[in]     err   The current error value.
 * @param[in]     dt_s  The time step in seconds.
 * @return float        The PID output value.
 */
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
