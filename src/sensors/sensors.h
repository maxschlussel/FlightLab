#pragma once

#include "src/sensors/altimeter.h"
#include "src/sensors/gps.h"
#include "src/sensors/imu.h"
#include "src/sensors/pitot_tube.h"


typedef struct {
    AltimeterSensor altimeterSensor;
    GPS gps;
    IMUSensor imuSensor;
    PitotTube pitotTube;
} Sensors;


Sensors initSensors(void);

void readSensors(StateVector* X, Sensors* sensors);
