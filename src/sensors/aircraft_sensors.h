#pragma once

#include "src/core/state_vector.h"
#include "src/actuators/actuators.h"
#include "src/core/aircraft_params.h"
#include "src/math/vector.h"
#include "src/sensors/altimeter.h"
#include "src/sensors/gps.h"
#include "src/sensors/imu.h"
#include "src/sensors/magnetometer.h"
#include "src/sensors/pitot_tube.h"

#include "src/core/control_vector.h"


/**
 * @brief Represents a collection of all aircraft simulated sensors.
 */
typedef struct {
    AltimeterSensor altimeterSensor;
    GPS gps;
    IMUSensor imuSensor;
    PitotTube pitotTube;
    Magnetometer mag;
} Sensors;


/**
 * @brief Contains pointers to all of the structures needed to compute all 
 *        aircraft sensor readings at the current time t.
 * 
 * @param X         State vector at current time t.
 * @param actuators Flight controls actuators from previous loop, t-1. Not yet updated 
 *                  for current loop at time t.
 * @param acParams  Aircraft parameters.
 */
typedef struct {
    const StateVector* X;
    const Actuators* actuators;
    const AircraftParams* acParams;
    double dt;
} SensorInput;

void readSensors(const SensorInput* sensorInput, Sensors* sensors);
