#include <math.h>

#include "src/core/constants.h"
#include "src/estimators/complementary_filter.h"
#include "src/estimators/simple_estimator.h"
#include "src/math/dcm.h"
#include "src/math/matrix.h"
#include "src/math/utils.h"
#include "src/math/vector.h"

#include "src/io/logger.h"

void estimateStateSimple(const Sensors* sensors, StateVector* X_est, double dt){
    Vector3 w_b_est  = sensors->imuSensor.gyro.data;

    Vector3 eulerAngles_est = estimateAttitudeCF(sensors, X_est, dt);
    
    Vector3 vel_b_est = estimateVelCF(&eulerAngles_est, sensors, X_est, dt);
    
    Vector3 pos_est = estimatePosCF(&eulerAngles_est, &vel_b_est, sensors, X_est, dt);
    
    X_est->u = vel_b_est.x;
    X_est->v = vel_b_est.y;
    X_est->w = vel_b_est.z;
    X_est->p = w_b_est.x;
    X_est->q = w_b_est.y;
    X_est->r = w_b_est.z;
    X_est->phi   = eulerAngles_est.x;
    X_est->theta = eulerAngles_est.y;
    X_est->psi   = eulerAngles_est.z;
    X_est->x = pos_est.x;
    X_est->y = pos_est.y;
    X_est->z = pos_est.z;

    // Log estimated states
    logger.data[LOG_X_EST_U] = X_est->u;
    logger.data[LOG_X_EST_V] = X_est->v;
    logger.data[LOG_X_EST_W] = X_est->w;
    logger.data[LOG_X_EST_P] = X_est->p;
    logger.data[LOG_X_EST_Q] = X_est->q;
    logger.data[LOG_X_EST_R] = X_est->r;
    logger.data[LOG_X_EST_PHI]   = X_est->phi;
    logger.data[LOG_X_EST_THETA] = X_est->theta;
    logger.data[LOG_X_EST_PSI]   = X_est->psi;
    logger.data[LOG_X_EST_X] = X_est->x;
    logger.data[LOG_X_EST_Y] = X_est->y;
    logger.data[LOG_X_EST_Z] = X_est->z;
}


/**
 * @brief Estimates the current attitude (Roll, Pitch, Yaw) by fusing integrated gyroscope and 
 * accelerometer sensor data using a Complementary Filter (CF).
 *
 * @param sensors   Pointer to the structure holding raw sensor readings.
 * @param X_est     Pointer to the current estimated StateVector structure.
 * @param dt        The time elapsed since the last update, in seconds.
 * @return          Vector3 A structure containing the new, filtered Euler angles in radians: 
 */
Vector3 estimateAttitudeCF(const Sensors* sensors, StateVector* X_est, double dt){
    // [0] Define useful quantities
    const double f_cutoff = 0.5;  // Hz
    const double tau = 1 / (twoPi * f_cutoff);
    const double alphaCF = tau / (tau + dt);

    const Vector3* gyro  = &(sensors->imuSensor.gyro.data);
    const Vector3* accel = &(sensors->imuSensor.accel.data);
    const Vector3* mag   = &(sensors->mag.data);

    double phi   = X_est->phi;
    double theta = X_est->theta;
    double psi   = X_est->psi;
    Vector3 w_b  = *gyro;  // Angular rates in body frame - omega_b

    // [1] Calculate estimated angles from Gyro
    double H[3][3] = {
        {1, sin(phi)*tan(theta), cos(phi)*tan(theta)},
        {0,            cos(phi),           -sin(phi)},
        {0, sin(phi)/cos(theta), cos(phi)/cos(theta)}
    };
    Vector3 euler_dot = mat3_mult_vec3(H, w_b);

    double phiGyro   = phi   + euler_dot.x * dt;
    double thetaGyro = theta + euler_dot.y * dt;
    double psiGyro   = psi   + euler_dot.z * dt;

    // [2] Calculate estimated anglesfrom Accel
    double ax = -accel->x;
    double ay = -accel->y;
    double az = -accel->z;

    double phiAccel = atan2(ay, az);
    double thetaAccel = atan2(ax, sqrt(ay*ay + az*az));
    
    // [3] Apply complementary filter blend to phi and theta
    phi     = phiGyro;//complementaryFilter(phiGyro, phiAccel, alphaCF);
    theta   = thetaGyro;//complementaryFilter(thetaGyro, thetaAccel, alphaCF);
    
    // [4] Calculate estimaged heading from Magnetometer and apply complementary filter blend to psi
    double psiMag = computeHeadingFromMag(mag, phi, theta);
    psi     = complementaryFilter(psiGyro, psiMag, alphaCF);

    phi   = wrapAngle(phi, -pi, pi);
    theta = wrapAngle(theta, -pi, pi);
    psi   = wrapAngle(psi, -pi, pi);

    return (Vector3) {phi, theta, psi};
}


double computeHeadingFromMag(const Vector3* mag, double phi_est, double theta_est) {
    double R_e2b[3][3];  // Earth (NED) 2 body dcm
    getRotationMatrix(phi_est, theta_est, 0, R_e2b);
    double R_b2e[3][3];  // Body 2 Earth (NED) dcm
    mat3_transpose(R_e2b, R_b2e);

    Vector3 B_ned = mat3_mult_vec3(R_b2e, *mag);

    double psi_mag = atan2(-B_ned.y, B_ned.x);

    psi_mag = wrapAngle(psi_mag, 0, twoPi);

    return psi_mag;
}


/**
 * @brief Estimates the current body frame velocity (u, v, w) by fusing integrated accelorometer and 
 * gps sensor data using a Complementary Filter (CF).
 *
 * @param eulerAngles_est   Pointer to a Vector3 containing the updated estimated euler angles.
 * @param sensors           Pointer to the structure holding raw sensor readings.
 * @param X_est             Pointer to the current estimated StateVector structure.
 * @param dt                The time elapsed since the last update, in seconds.
 * @return                  Vector3 A structure containing the new, filtered body frame velocity.
 */
Vector3 estimateVelCF(const Vector3* eulerAngles_est, const Sensors* sensors, StateVector* X_est, double dt){
    // [0] Define useful quantities
    const double f_cutoff = 0.25;  // Hz
    const double tau = 1 / (twoPi * f_cutoff);
    const double alphaCF = tau / (tau + dt);
    
    double phi   = eulerAngles_est->x;
    double theta = eulerAngles_est->y;
    double psi   = eulerAngles_est->z;

    Vector3 vel_b_est = {X_est->u, X_est->v, X_est->w};

    Vector3 velGPS = sensors->gps.vel.data;

    // [1] Compute accel and vel in NED frame  
    double R_e2b[3][3];  // Earth (NED) 2 body dcm
    getRotationMatrix(phi, theta, psi, R_e2b);
    double R_b2e[3][3];  // Body 2 Earth (NED) dcm
    mat3_transpose(R_e2b, R_b2e);

    Vector3 vel_NED = mat3_mult_vec3(R_b2e, vel_b_est);
    Vector3 accel_NED = mat3_mult_vec3(R_b2e, sensors->imuSensor.accel.data);
    
    // [2] Accel -> real world: Add grav back into accel reading
    Vector3 grav = {0, 0, g};
    accel_NED = vec3_add(accel_NED, grav);
    
    // [3] Apply complementary filter blend
    Vector3 deltaAccel_NED = vec3_scale(accel_NED, dt);
    Vector3 velAccel = vec3_add(vel_NED, deltaAccel_NED);
    
    double velX = complementaryFilter(velAccel.x, velGPS.x, alphaCF);
    double velY = complementaryFilter(velAccel.y, velGPS.y, alphaCF);
    double velZ = complementaryFilter(velAccel.z, velGPS.z, alphaCF);
    
    vel_b_est = mat3_mult_vec3(R_e2b, (Vector3) {velX, velY, velZ});
    return vel_b_est;
}


/**
 * @brief Estimates the current position in NED (x, y, z) by fusing integrated velocity and 
 * gps sensor data using a Complementary Filter (CF).
 *
 * @param eulerAngles_est   Pointer to a Vector3 containing the updated estimated euler angles.
 * @param vel_b_est         Pointer to a Vector3 containing the updated estimated body frame velocities.
 * @param sensors           Pointer to the structure holding raw sensor readings.
 * @param X_est             Pointer to the current estimated StateVector structure.
 * @param dt                The time elapsed since the last update, in seconds.
 * @return                  Vector3 A structure containing the new, filtered position in NED. 
 */
Vector3 estimatePosCF(const Vector3* eulerAngles_est, const Vector3* vel_b_est, const Sensors* sensors, StateVector* X_est, double dt){
    // [0] Define useful quantities
    const double f_cutoff = 0.25;  // Hz
    const double tau = 1 / (twoPi * f_cutoff);
    const double alphaCF = tau / (tau + dt);

    double phi   = eulerAngles_est->x;
    double theta = eulerAngles_est->y;
    double psi   = eulerAngles_est->z;

    Vector3 pos_est = {X_est->x, X_est->y, X_est->z};

    // [1] Compute vel in NED frame  
    double R_e2b[3][3];  // Earth (NED) 2 body dcm
    getRotationMatrix(phi, theta, psi, R_e2b);
    double R_b2e[3][3];  // Body 2 Earth (NED) dcm
    mat3_transpose(R_e2b, R_b2e);
    
    Vector3 vel_NED = mat3_mult_vec3(R_b2e, *vel_b_est);

    Vector3 posVeldx = vec3_scale(vel_NED, dt);

    Vector3 posNED = vec3_add(pos_est, posVeldx);

    double posX = complementaryFilter(posNED.x, sensors->gps.pos.data.x, alphaCF);
    double posY = complementaryFilter(posNED.y, sensors->gps.pos.data.y, alphaCF);
    double posZ = complementaryFilter(posNED.z, sensors->gps.pos.data.z, alphaCF);

    return (Vector3) {posX, posY, posZ};
}