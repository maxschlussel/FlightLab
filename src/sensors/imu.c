#include <math.h>

#include "src/core/constants.h"
#include "src/math/dcm.h"
#include "src/math/matrix.h"
#include "src/math/utils.h"
#include "src/sensors/imu.h"

#include "src/io/logger.h"

/**
 * @brief Reads the simulated IMU sensor.
 *
 * @param[in]   X       Pointer to the current state vector.
 * @param[in]   Xdot    Pointer to the current state vector.
 * @param[out]  imu     Pointer to the imu sensor struct
 */
void readIMUSensor(const StateVector* X, const double* Xdot, double dt, IMUSensor* imu){
    // [1] Calculate Gyroscope reading
    imu->gyro.bias.x += randNoise(imu->gyro.sigmaWalk * sqrt(dt));
    imu->gyro.bias.y += randNoise(imu->gyro.sigmaWalk * sqrt(dt));
    imu->gyro.bias.z += randNoise(imu->gyro.sigmaWalk * sqrt(dt));
    
    imu->gyro.data.x = X->p + imu->gyro.bias.x + randNoise(imu->gyro.sigmaNoise);
    imu->gyro.data.y = X->q + imu->gyro.bias.y + randNoise(imu->gyro.sigmaNoise);
    imu->gyro.data.z = X->r + imu->gyro.bias.z + randNoise(imu->gyro.sigmaNoise);
    
    // [2] Calculate Accelerometer readings
    Vector3 accel_xdot = {Xdot[0], Xdot[1], Xdot[2]};

    double R_e2b[3][3];  // Earth (NED) 2 body dcm
    getRotationMatrix(X->phi, X->theta, X->psi, R_e2b);

    Vector3 g_ned = {0.0, 0.0, g};
    Vector3 g_b = mat3_mult_vec3(R_e2b, g_ned);

    Vector3 accel_b = vec3_sub(accel_xdot, g_b);

    imu->accel.bias.x += randNoise(imu->accel.sigmaWalk * sqrt(dt));
    imu->accel.bias.y += randNoise(imu->accel.sigmaWalk * sqrt(dt));
    imu->accel.bias.z += randNoise(imu->accel.sigmaWalk * sqrt(dt));

    imu->accel.data.x = accel_b.x + imu->accel.bias.x + randNoise(imu->accel.sigmaNoise);
    imu->accel.data.y = accel_b.y + imu->accel.bias.y + randNoise(imu->accel.sigmaNoise);
    imu->accel.data.z = accel_b.z + imu->accel.bias.z + randNoise(imu->accel.sigmaNoise);

    logger.data[LOG_SNS_IMU_GYRO_X] = imu->gyro.data.x;
    logger.data[LOG_SNS_IMU_GYRO_Y] = imu->gyro.data.y;
    logger.data[LOG_SNS_IMU_GYRO_Z] = imu->gyro.data.z;
    logger.data[LOG_SNS_IMU_ACCEL_X] = imu->accel.data.x;
    logger.data[LOG_SNS_IMU_ACCEL_Y] = imu->accel.data.y;
    logger.data[LOG_SNS_IMU_ACCEL_Z] = imu->accel.data.z;
}
