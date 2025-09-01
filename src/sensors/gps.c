#include <math.h>
#include <stdlib.h>

#include "src/math/utils.h"
#include "src/sensors/gps.h"


void readGPS(const StateVector* X, GPS* gps){
    gps->pos.x = X->x + randNoise(10);
    gps->pos.y = X->y + randNoise(10);
    gps->pos.z = X->z + randNoise(10);
}
