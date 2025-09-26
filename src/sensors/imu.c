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
    imu->gyro[0] = X->p + randNoise(0.001);
    imu->gyro[1] = X->q + randNoise(0.001);
    imu->gyro[2] = X->r + randNoise(0.001);

    imu->accel[0] = 0.0 + randNoise(0.01);
    imu->accel[1] = 0.0 + randNoise(0.01);
    imu->accel[2] = g + randNoise(0.01);

    logger.data[LOG_SNS_IMU_GYRO_X] = imu->gyro[0];
    logger.data[LOG_SNS_IMU_GYRO_Y] = imu->gyro[1];
    logger.data[LOG_SNS_IMU_GYRO_Z] = imu->gyro[2];
    logger.data[LOG_SNS_IMU_ACCEL_X] = imu->accel[0];
    logger.data[LOG_SNS_IMU_ACCEL_Y] = imu->accel[1];
    logger.data[LOG_SNS_IMU_ACCEL_Z] = imu->accel[2];
}
