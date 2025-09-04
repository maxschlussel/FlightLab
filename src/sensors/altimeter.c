#include <math.h>
#include <stdlib.h>

#include "src/math/utils.h"
#include "src/sensors/altimeter.h"

#include "src/io/logger.h"


void readAltimeterSensor(const StateVector* X, AltimeterSensor* altemeterSensor){
    altemeterSensor->alt = X->z + randNoise(5);

    logger.data[LOG_SNS_ALT] = altemeterSensor->alt;
}
