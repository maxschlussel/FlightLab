#include <math.h>

#include "src/core/constants.h"
#include "src/math/utils.h"
#include "src/sensors/imu.h"

#include "src/io/logger.h"

/**
 * @brief Reads the simulated IMU sensor.
 *
 * @param[in]   X       Pointer to the current state vector.
 * @param[out]  imu     Pointer to the imu sensor struct
 */
void readIMUSensor(const StateVector* X, IMUSensor* imu){
    imu->gyro.x = X->p + randNoise(0.001);
    imu->gyro.y = X->q + randNoise(0.001);
    imu->gyro.z = X->r + randNoise(0.001);

    imu->accel.x = 0.0 + randNoise(0.01);
    imu->accel.y = 0.0 + randNoise(0.01);
    imu->accel.z = g + randNoise(0.01);

    logger.data[LOG_SNS_IMU_GYRO_X] = imu->gyro.x;
    logger.data[LOG_SNS_IMU_GYRO_Y] = imu->gyro.y;
    logger.data[LOG_SNS_IMU_GYRO_Z] = imu->gyro.z;
    logger.data[LOG_SNS_IMU_ACCEL_X] = imu->accel.x;
    logger.data[LOG_SNS_IMU_ACCEL_Y] = imu->accel.y;
    logger.data[LOG_SNS_IMU_ACCEL_Z] = imu->accel.z;
}
