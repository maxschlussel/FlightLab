#pragma once

// Development in progress...

/**
 * @brief Struct to hold the state and gains of a PID controller.
 */
typedef struct {
    double kp;          // Proportial gain
    double ki;          // Integral gain
    double kd;          // Derivative gain

    double integral;    // Accumulated integral err
    double prevErr;     // Previous error value for derivative

    double outMin;      // Minimum output value
    double outMax;      // Maximum output value
} PID;

PID initPID(double kp, double ki, double kd, double outMin, double outMax);

float computePID(PID* pid, double err, double dt_s);
