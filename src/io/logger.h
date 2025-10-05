#pragma once

#include <stdio.h>

#define LOGGER_SIZE 128


typedef enum {
    LOG_SNS_ALT,
    LOG_SNS_GPS_POS_X,
    LOG_SNS_GPS_POS_Y,
    LOG_SNS_GPS_POS_Z,
    LOG_SNS_GPS_VEL_X,
    LOG_SNS_GPS_VEL_Y,
    LOG_SNS_GPS_VEL_Z,
    LOG_SNS_IMU_GYRO_X,
    LOG_SNS_IMU_GYRO_Y,
    LOG_SNS_IMU_GYRO_Z,
    LOG_SNS_IMU_ACCEL_X,
    LOG_SNS_IMU_ACCEL_Y,
    LOG_SNS_IMU_ACCEL_Z,
    LOG_SNS_PITOT_VEL,
    LOG_SNS_MAG_X,
    LOG_SNS_MAG_Y,
    LOG_SNS_MAG_Z,
    LOG_U_CMD_DA,
    LOG_U_CMD_DE,
    LOG_U_CMD_DR,
    LOG_U_CMD_DTHR1,
    LOG_U_CMD_DTHR2,
    LOG_SRVO_AILERON_POS,
    LOG_SRVO_ELEVATOR_POS,
    LOG_SRVO_RUDDER_POS,
    LOG_SRVO_THROTTLE1_VAL,
    LOG_SRVO_THROTTLE2_VAL,
    LOG_FORCES_F_TOT_X,
    LOG_FORCES_F_TOT_Y,
    LOG_FORCES_F_TOT_Z,
    LOG_FORCES_F_AERO_X,
    LOG_FORCES_F_AERO_Y,
    LOG_FORCES_F_AERO_Z,
    LOG_FORCES_F_PROP_X,
    LOG_FORCES_F_PROP_Y,
    LOG_FORCES_F_PROP_Z,
    LOG_FORCES_F_GRAV_X,
    LOG_FORCES_F_GRAV_Y,
    LOG_FORCES_F_GRAV_Z,
    LOG_MOMENTS_M_TOT_X,
    LOG_MOMENTS_M_TOT_Y,
    LOG_MOMENTS_M_TOT_Z,
    LOG_MOMENTS_M_AERO_X,
    LOG_MOMENTS_M_AERO_Y,
    LOG_MOMENTS_M_AERO_Z,
    LOG_MOMENTS_M_PROP_X,
    LOG_MOMENTS_M_PROP_Y,
    LOG_MOMENTS_M_PROP_Z,
    LOG_QINF,
    LOG_ALPHA,
    LOG_BETA,
    LOG_AERO_COEF_CL,
    LOG_AERO_COEF_CD,
    LOG_AERO_COEF_CY,
    LOG_AERO_COEF_Cl,
    LOG_AERO_COEF_Cm,
    LOG_AERO_COEF_Cn,
    LOG_XDOT_U,
    LOG_XDOT_V,
    LOG_XDOT_W,
    LOG_XDOT_P,
    LOG_XDOT_Q,
    LOG_XDOT_R,
    LOG_XDOT_PHI,
    LOG_XDOT_THETA,
    LOG_XDOT_PSI,
    LOG_XDOT_X,
    LOG_XDOT_Y,
    LOG_XDOT_Z,
    LOG_X_U,
    LOG_X_V,
    LOG_X_W,
    LOG_X_P,
    LOG_X_Q,
    LOG_X_R,
    LOG_X_PHI,
    LOG_X_THETA,
    LOG_X_PSI,
    LOG_X_X,
    LOG_X_Y,
    LOG_X_Z,
    LOG_X_EST_U,
    LOG_X_EST_V,
    LOG_X_EST_W,
    LOG_X_EST_P,
    LOG_X_EST_Q,
    LOG_X_EST_R,
    LOG_X_EST_PHI,
    LOG_X_EST_THETA,
    LOG_X_EST_PSI,
    LOG_X_EST_X,
    LOG_X_EST_Y,
    LOG_X_EST_Z,
    LOG_DEBUG1,
    LOG_DEBUG2,
    LOG_DEBUG3,
    LOG_DEBUG4,
    LOG_DEBUG5,
    LOG_COUNT
} LogSignal;


/**
 * @brief Represents the logger system for data recording.
 */
typedef struct {
    FILE* fp;
    double data[LOGGER_SIZE];
}Logger;


// Global logger instance
extern Logger logger;

extern const char* logSignalNames[LOG_COUNT];

void loggerInit(const char* filename);

void loggerLogHeader(void);

void loggerLogStep(double simTime);

void loggerClear(void);

void loggerClose(void);
