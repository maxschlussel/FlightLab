#pragma once


typedef struct {
    double kp;  // Proportial gain
    double ki;  // Integral gain
    double kd;  // Derivative gain

    double integral;    // Accumulated integral
    double prevErr;     // Previous error for derivative

    double outMin;
    double outMax;
} PID;

PID initPID(double kp, double ki, double kd, double outMin, double outMax);

float computePID(PID* pid, double err, double dt_s);
