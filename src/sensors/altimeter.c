#include <math.h>
#include <stdlib.h>

#include "src/math/utils.h"
#include "src/sensors/altimeter.h"


void readAltimeterSensor(const StateVector* X, AltimeterSensor* altemeterSensor){
    altemeterSensor->alt = X->z + randNoise(5);
}
