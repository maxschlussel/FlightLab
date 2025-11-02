#pragma once

#include "src/core/state_vector.h"
#include "src/sensors/aircraft_sensors.h"
#include "src/dynamics/aerodynamics.h"

#define N_EKF_STATE 12
#define N_EKF_MEAS 15

// Module in development...

typedef struct {
    double X[N_EKF_STATE];                  // Current state estimate
    double P[N_EKF_STATE][N_EKF_STATE];     // State covariance matrix
    double Q[N_EKF_STATE][N_EKF_STATE];     // Process noise covariance
    double R[N_EKF_MEAS][N_EKF_MEAS];       // Measurement noise covariance (size depends on sensors used)
    
    double F[N_EKF_STATE][N_EKF_STATE];     // Jacobian of process model wrt state
    double F_T[N_EKF_STATE][N_EKF_STATE];   // Transpose of F
    double H[N_EKF_MEAS][N_EKF_MEAS];       // Jacobian of measurement model wrt state
    double K[N_EKF_STATE][N_EKF_MEAS];      // Kalman gain matrix

    double y[N_EKF_MEAS];                   // Measurment residial/ innovation (error)
    double Xdot[N_EKF_STATE];               // State derivative (from dynamics)
    double Z_pred[N_EKF_MEAS];              // Predicted measurement vector

    double dt;

    AeroData aeroData;
    
    const AircraftModel* acModel; // Pointer to aircraft parameters
    const Sensors* sensors;         // Pointer to current sensor readings
    const Actuators* actuators;         // Pointer to current sensor readings

} EKF;

EKF initEKF(const AircraftModel* acModel, const Actuators* actuators);

void estimateStateEKF(EKF* ekf, Sensors* sensors, double dt);