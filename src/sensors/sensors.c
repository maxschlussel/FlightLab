#include "src/math/vector.h"
#include "src/sensors/sensors.h"

void readSensors(StateVector* X, Sensors* sensors){
    readAltimeterSensor(X, &(sensors->altimeterSensor));
    readGPS(X, &(sensors->gps));
    readIMUSensor(X, &(sensors->imuSensor));
    readPitotTube(X, &(sensors->pitotTube));    
}

Sensors initSensors(void){
    Sensors sensors = {
        .altimeterSensor.alt = 0.0,
        .gps.pos = {0.0},
        .imuSensor.accel = {0.0},
        .imuSensor.gyro = {0.0},
        .pitotTube.vel = {0.0}
    };
    return sensors;
}