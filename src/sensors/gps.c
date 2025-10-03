#include <math.h>
#include <stdlib.h>

#include "src/math/dcm.h"
#include "src/math/matrix.h"
#include "src/math/utils.h"
#include "src/sensors/gps.h"

#include "src/io/logger.h"


/**
 * @brief Reads the simulated GPS sensor.
 *
 * @param[in]   X       Pointer to the current state vector.
 * @param[out]  gps     Pointer to the GPS sensor struct
 */
void readGPS(const StateVector* X, GPS* gps){
    Vector3 V_b = {X->u, X->v, X->w}; // Velocities in body frame - V_b

    // Position
    gps->pos.x = X->x + randNoise(10);
    gps->pos.y = X->y + randNoise(10);
    gps->pos.z = X->z + randNoise(10);
    
    // Velocity
    double R_e2b[3][3];  // Earth 2 body dcm
    getRotationMatrix(X->phi, X->theta, X->psi, R_e2b);
    double R_b2e[3][3];
    mat3_transpose(R_e2b, R_b2e);
    Vector3 dPos = mat3_mult_vec3(R_b2e, V_b);
    
    gps->vel.x = dPos.x + randNoise(10);
    gps->vel.y = dPos.y + randNoise(10);
    gps->vel.z = dPos.z + randNoise(10);

    logger.data[LOG_SNS_GPS_POS_X] = gps->pos.x;
    logger.data[LOG_SNS_GPS_POS_Y] = gps->pos.y;
    logger.data[LOG_SNS_GPS_POS_Z] = gps->pos.z;
    logger.data[LOG_SNS_GPS_VEL_X] = gps->vel.x;
    logger.data[LOG_SNS_GPS_VEL_Y] = gps->vel.y;
    logger.data[LOG_SNS_GPS_VEL_Z] = gps->vel.z;
}
