#include <math.h>

#include "src/math/dcm.h"
#include "src/math/matrix.h"
#include "src/math/utils.h"
#include "src/sensors/magnetometer.h"

#include "src/io/logger.h"

/**
 * @brief Reads the simulated magnetometer sensor.
 *
 * @param[out]  mag Pointer to the magnetometer sensor.
 */
void readMagnetometer(const StateVector* X, Magnetometer* mag, double dt){
    double R_e2b[3][3];  // Earth (NED) 2 body dcm
    getRotationMatrix(X->phi, X->theta, X->psi, R_e2b);
    Vector3 B_body = mat3_mult_vec3(R_e2b, B_ned);

    mag->bias.x += randNoise(mag->sigmaWalk * sqrt(dt));
    mag->bias.y += randNoise(mag->sigmaWalk * sqrt(dt));
    mag->bias.z += randNoise(mag->sigmaWalk * sqrt(dt));
    
    mag->data.x = B_body.x + mag->bias.x + randNoise(mag->sigmaNoise);
    mag->data.y = B_body.y + mag->bias.y + randNoise(mag->sigmaNoise);
    mag->data.z = B_body.z + mag->bias.z + randNoise(mag->sigmaNoise);
    
    logger.data[LOG_SNS_MAG_X] = mag->data.x;
    logger.data[LOG_SNS_MAG_Y] = mag->data.y;
    logger.data[LOG_SNS_MAG_Z] = mag->data.z;
}
