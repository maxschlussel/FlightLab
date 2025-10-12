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
    
    readGPS(X, &(sensors->gps), dt);
    
    readPitotTube(X, &(sensors->pitotTube));   
    
    readMagnetometer(X, &(sensors->mag), dt);
    
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

/**
 * Convert Sensor struct to 1D array
 * z=[p,q,r,ax​,ay​,az​,x,y,z,u,v,w,ψ]
 */
void sensors_to_array(const Sensors* sensors, double arr[N_SENSOR_DATA]){
    arr[0] = sensors->imuSensor.gyro.data.x;
    arr[1] = sensors->imuSensor.gyro.data.y;
    arr[2] = sensors->imuSensor.gyro.data.z;
    arr[3] = sensors->imuSensor.accel.data.x;
    arr[4] = sensors->imuSensor.accel.data.y;
    arr[5] = sensors->imuSensor.accel.data.z;
    arr[6] = sensors->gps.pos.data.x;
    arr[7] = sensors->gps.pos.data.y;
    arr[8] = sensors->gps.pos.data.z;
    arr[9] = sensors->gps.vel.data.x;
    arr[10] = sensors->gps.vel.data.y;
    arr[11] = sensors->gps.vel.data.z;
    arr[12] = sensors->pitotTube.vel;
    arr[13] = sensors->mag.data.x; // TODO: FIX
    arr[14] = sensors->altimeterSensor.alt;
}
