#include "src/math/vector.h"
#include "src/sensors/sensors.h"

#include "src/io/logger.h"

/**
 * @brief Reads data from all simulated sensors.
 *
 * @param[in]   X       Pointer to the true state vector of the aircraft.
 * @param[out]  sensors Pointer to the `Sensors` struct where all the
 *                      sensor measurement data will be stored.
 */
void readSensors(StateVector* X, Sensors* sensors){
    readAltimeterSensor(X, &(sensors->altimeterSensor));
    readGPS(X, &(sensors->gps));
    readIMUSensor(X, &(sensors->imuSensor));
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


/**
 * @brief Initializes the sensor data to zero.
 *
 * This function creates a `Sensors` struct and initializes all of its
 * members—altimeter, GPS, IMU, and pitot tube—to zero values. 
 *
 * @return A `Sensors` struct with all data fields initialized to zero.
 */
Sensors initSensors(void){
    Sensors sensors = {
        .altimeterSensor.alt = 0.0,
        .gps.pos = {0.0},
        .imuSensor.accel = {0.0},
        .imuSensor.gyro = {0.0},
        .pitotTube.vel = 0.0
    };
    return sensors;
}