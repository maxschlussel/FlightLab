#include <stdlib.h>

#include "src/io/logger.h"


// Global logger instance
Logger logger;


const char* logSignalNames[LOG_COUNT] = {
    "sns_alt",
    "sns_gps_pos_x",
    "sns_gps_pos_y",
    "sns_gps_pos_z",
    "sns_gps_vel_x",
    "sns_gps_vel_y",
    "sns_gps_vel_z",
    "sns_imu_gyro_x",
    "sns_imu_gyro_y",
    "sns_imu_gyro_z",
    "sns_imu_accel_x",
    "sns_imu_accel_y",
    "sns_imu_accel_z",
    "sns_pitot_vel",
    "sns_mag_x",
    "sns_mag_y",
    "sns_mag_z",
    "u_cmd_da",
    "u_cmd_de",
    "u_cmd_dr",
    "u_cmd_dthr1",
    "u_cmd_dthr2",
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
    "X_est_u",
    "X_est_v",
    "X_est_w",
    "X_est_p",
    "X_est_q",
    "X_est_r",
    "X_est_phi",
    "X_est_theta",
    "X_est_psi",
    "X_est_x",
    "X_est_y",
    "X_est_z",
    "debug1",
    "debug2",
    "debug3",
    "debug4",
    "debug5"
};


/**
 * @brief Initializes the logger and opens the specified log file.
 *
 * @param[in] filename Pointer to a null-terminated string containing the name of the log file.
 */
void loggerInit(const char* filename){
    logger.fp = fopen(filename, "w");

    if (!logger.fp) {
        perror("Error: could not open logger file.\n");
        exit(1);
    }

    loggerClear();

    loggerLogHeader();
}


/**
 * @brief Writes the header row to the log file.
 */
void loggerLogHeader(void){
    fprintf(logger.fp, "time");

    for (int i = 0; i < LOG_COUNT; i++) {
        fprintf(logger.fp, ", %s", logSignalNames[i]);
    }

    fprintf(logger.fp, "\n");    
}


/**
 * @brief Logs a single row of data to the log file.
 *
 * This function records the current simulation time and all data
 * from the `logger.data` array as a new, comma-separated row in the log file.
 *
 * @param[in] simTime_s The current simulation time in seconds.
 */
void loggerLogStep(double simTime_s){
    if (!logger.fp) return;

    // Log time
    fprintf(logger.fp, "%.6f", simTime_s);

    // Log all data
    for (int i = 0; i < LOG_COUNT; i++){
        fprintf(logger.fp, ", %.6f", logger.data[i]);
    }

    fprintf(logger.fp, "\n");
}

/**
 * Clears the logger data to 0.
 */
void loggerClear(void){
    for (int i = 0; i < LOG_COUNT; i++) logger.data[i] = 0.0;
}

/**
 * Closes the log file.
 */
void loggerClose(void){
    if (logger.fp) fclose(logger.fp);
}
