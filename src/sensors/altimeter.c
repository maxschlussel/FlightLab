#include <math.h>
#include <stdlib.h>

#include "src/math/utils.h"
#include "src/sensors/altimeter.h"

#include "src/io/logger.h"


/**
 * @brief Reads the simulated altimeter sensor.
 *
 * @param[in]   X               Pointer to the current state vector.
 * @param[out]  altemeterSensor Pointer to the altimeter sensor struct
 */
void readAltimeterSensor(const StateVector* X, AltimeterSensor* altemeterSensor){
    // Module in development...
    altemeterSensor->alt = X->z + randNoise(5);

    logger.data[LOG_SNS_ALT] = altemeterSensor->alt;
}
