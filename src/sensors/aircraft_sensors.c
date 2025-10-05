#include "src/dynamics/eom.h"
#include "src/dynamics/forces.h"
#include "src/math/vector.h"
#include "src/sensors/aircraft_sensors.h"

#include "src/io/logger.h"

/**
 * @brief Calculates data readings for all simulated sensors.
 *
 * @param[in]   sensorInput Pointer to the 'sensorInput struct'.
 * @param[out]  sensors     Pointer to the `Sensors` struct where all the
 *                          sensor measurement data will be stored.
 */
void readSensors(const SensorInput* sensorInput, Sensors* sensors){
    // [0] Unpack sensor input params
    const StateVector* X = sensorInput->X;
    const Actuators* actuators = sensorInput->actuators;
    const AircraftParams* acParams = sensorInput->acParams;
    double dt = sensorInput->dt;

    Vector3 F = {0.0};
    Vector3 M = {0.0};
    double Xdot[12] = {0.0};

    // [1] Compute sensor input value Xdot at current time
    computeForcesAndMoments(X, actuators, acParams, &F, &M);
    computeStateDerivative(X, acParams, &F, &M, Xdot);

    // [2] Compute sensor readings
    readIMUSensor(X, Xdot, dt, &(sensors->imuSensor));

    readAltimeterSensor(X, &(sensors->altimeterSensor));
    
    readGPS(X, &(sensors->gps));
    
    readPitotTube(X, &(sensors->pitotTube));   
    
    // Log ground truth states
    logger.data[LOG_X_U] = X->u;
    logger.data[LOG_X_V] = X->v;
    logger.data[LOG_X_W] = X->w;
    logger.data[LOG_X_P] = X->p;
    logger.data[LOG_X_Q] = X->q;
    logger.data[LOG_X_R] = X->r;
    logger.data[LOG_X_PHI]   = X->phi;
    logger.data[LOG_X_THETA] = X->theta;
    logger.data[LOG_X_PSI]   = X->psi;
    logger.data[LOG_X_X] = X->x;
    logger.data[LOG_X_Y] = X->y;
    logger.data[LOG_X_Z] = X->z;
}
