#include <stdlib.h>

#include "src/core/constants.h"
#include "src/io/logger.h"


// Define the global logger
Logger logger;


const char* LogSignalNames[LOG_COUNT] = {
    "sns_alt",
    "sns_gps_pos_x",
    "sns_gps_pos_y",
    "sns_gps_pos_z",
    "sns_imu_gyro_x",
    "sns_imu_gyro_y",
    "sns_imu_gyro_z",
    "sns_imu_accel_x",
    "sns_imu_accel_y",
    "sns_imu_accel_z",
    "sns_pitot_vel",
    "u_da",
    "u_de",
    "u_dr",
    "u_dthr1",
    "u_dthr2",
    "srvo_aileron_pos",
    "srvo_elevator_pos",
    "srvo_rudder_pos",
    "srvo_throttle1_val",
    "srvo_throttle2_val",
    "forces_f_tot_x",
    "forces_f_tot_y",
    "forces_f_tot_z",
    "forces_f_aero_x",
    "forces_f_aero_y",
    "forces_f_aero_z",
    "forces_f_prop_x",
    "forces_f_prop_y",
    "forces_f_prop_z",
    "forces_f_grav_x",
    "forces_f_grav_y",
    "forces_f_grav_z",
    "moments_m_tot_x",
    "moments_m_tot_y",
    "moments_m_tot_z",
    "moments_m_aero_x",
    "moments_m_aero_y",
    "moments_m_aero_z",
    "moments_m_prop_x",
    "moments_m_prop_y",
    "moments_m_prop_z",
    "qinf",
    "alpha",
    "beta",
    "aero_coef_CL",
    "aero_coef_Cd",
    "aero_coef_Cy",
    "aero_coef_Cl",
    "aero_coef_Cm",
    "aero_coef_Cn",
    "Xdot_u",
    "Xdot_v",
    "Xdot_w",
    "Xdot_p",
    "Xdot_q",
    "Xdot_r",
    "Xdot_phi",
    "Xdot_theta",
    "Xdot_psi",
    "Xdot_x",
    "Xdot_y",
    "Xdot_z",
    "X_u",
    "X_v",
    "X_w",
    "X_p",
    "X_q",
    "X_r",
    "X_phi",
    "X_theta",
    "X_psi",
    "X_x",
    "X_y",
    "X_z",
    "debug1",
    "debug2",
    "debug3",
    "debug4",
    "debug5"
};


void loggerInit(const char* filename){
    logger.fp = fopen(filename, "w");

    if (!logger.fp) {
        perror("Error: could not open logger file.\n");
        exit(1);
    }

    loggerClear();

    loggerLogHeader();
}


void loggerLogHeader(void){
    fprintf(logger.fp, "time");

    for (int i = 0; i < LOG_COUNT; i++) {
        fprintf(logger.fp, ", %s", LogSignalNames[i]);
    }

    fprintf(logger.fp, "\n");    
}


void loggerLogStep(double simTime_s){
    if (!logger.fp) return;

    // Log time
    fprintf(logger.fp, "%.3f", simTime_s);

    // Log all data
    for (int i = 0; i < LOG_COUNT; i++){
        fprintf(logger.fp, ", %.3f", logger.data[i]);
    }

    fprintf(logger.fp, "\n");
}


void loggerClear(void){
    for (int i = 0; i < LOG_COUNT; i++) logger.data[i] = 0.0;
}


void loggerClose(void){
    if (logger.fp) fclose(logger.fp);
}
