#include <math.h>

#include "src/core/constants.h"
#include "src/math/utils.h"
#include "src/sensors/imu.h"


void readIMUSensor(const StateVector* X, IMUSensor* imu){
    imu->gyro[0] = X->p + randNoise(0.001);
    imu->gyro[1] = X->q + randNoise(0.001);
    imu->gyro[2] = X->r + randNoise(0.001);

    imu->accel[0] = 0.0 + randNoise(0.01);
    imu->accel[1] = 0.0 + randNoise(0.01);
    imu->accel[2] = g + randNoise(0.01);
}
