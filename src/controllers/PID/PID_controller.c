#include "src/controllers/PID/PID_controller.h"
#include "src/controllers/PID/PID.h"
#include "src/math/utils.h"

#include "src/io/logger.h"

/**
 * @brief Computes flight control commands using cascaded PID controllers.
 *
 * This function implements the flight control logic for an aircraft by utilizing
 * a set of cascaded PID controllers. It processes estimated state measurements,
 * guidance references, and aircraft parameters to generate control commands for
 * ailerons, elevator, rudder, and dual throttles.
 *
 * The control system is structured in two main loops:
 * - **Longitudinal Control:** An outer loop controls altitude, commanding a desired
 * pitch angle. An inner loop then controls pitch, commanding an elevator deflection.
 * Airspeed is controlled independently by commanding a throttle setting.
 * - **Lateral Control:** An outer loop controls heading, commanding a desired
 * roll angle. An inner loop then controls roll, commanding an aileron deflection.
 *
 * @param[in]     X_est             A pointer to the estimated StateVector, containing
 *                                  the aircraft's current measured state.
 * @param[in]     guidanceRefs      A pointer to the GuidanceRefs struct, containing
 *                                  the desired reference values for altitude,
 *                                  airspeed, and heading.
 * @param[in]     acParams          A pointer to the AircraftParams struct, containing
 *                                  physical properties of the aircraft.
 * @param[in,out] controlSystemPID  A pointer to the ControlSystemPID struct, which
 *                                  contains the PID controllers and will be updated
 *                                  with the new actuator commands.
 * @param[in]     dt_s              The time step in seconds since the last update.
 */
void computeFlightControlPID(StateVector* X_est, GuidanceRefs* guidanceRefs, AircraftParams* acParams, 
                             ControlSystemPID* controlSystemPID, double dt_s) {
    // [0] Defne Useful Quantities
    Vector3 w_b = {X_est->p, X_est->q, X_est->r}; // Angular rates in body frame - omega_b
    Vector3 V_b = {X_est->u, X_est->v, X_est->w}; // Velocities in body frame - V_b
    Vector3 U_123 = {controlSystemPID->U_cmd.da, 
                     controlSystemPID->U_cmd.de, 
                     controlSystemPID->U_cmd.dr};  // Primary controls
    
    PID* alt2PitchPID = &(controlSystemPID->alt2pitchPID);
    PID* pitch2elvPID = &(controlSystemPID->pitch2elvPID);
    PID* airspeed2throtPID = &(controlSystemPID->airspeed2throtPID);

    PID* heading2rollPID = &(controlSystemPID->heading2rollPID);
    PID* roll2aileronPID = &(controlSystemPID->roll2aileronPID);

    double velocity = vec3_norm(V_b);
    double Q = 0.5 * rho * pow(velocity, 2);  // Dynamic pressure
    double alpha = atan2(X_est->w, X_est->u);
    double beta = asin(X_est->v/velocity); //?????????

    double alt_meas = X_est->z;
    double pitch_meas = X_est->theta;
    double heading_meas = X_est->psi;
    double roll_meas = X_est->phi;
    double vel_meas = vec3_norm(V_b);

    double alt_ref = -1000; // guidanceRefs->refs.headingAltVel.vel // [m]
    double vel_ref = 85; // guidanceRefs->refs.headingAltVel.vel    // [m/s]
    double heading_ref = 20*deg2rad;  // guidanceRefs->refs.headingAltVel.heading    // [deg]

    // ---- Longitudinal ----
    // [1] Outer Loop - Alt to Pitch
    double alt_err = alt_ref - alt_meas;
    double pitchCmd = computePID(alt2PitchPID, alt_err, dt_s) + 5.0 * deg2rad;
    
    // [2] Inner Loop - Pitch to Elv
    double pitch_err = pitchCmd - pitch_meas;
    double elvCmd = computePID(pitch2elvPID, pitch_err, dt_s) - 10.2 * deg2rad;
    
    // [3] Airspeed to throttle
    double vel_err = vel_ref - vel_meas;
    double throtCmd = computePID(airspeed2throtPID, vel_err, dt_s) + 4.7 * deg2rad;
    
    // ---- Lateral ----
    // [1] Heading to Roll
    double heading_err = heading_ref - heading_meas;
    double rollCmd = computePID(heading2rollPID, heading_err, dt_s);
    
    // [2] Roll to Aileron
    double roll_err = rollCmd - roll_meas;
    double aileronCmd = computePID(roll2aileronPID, roll_err, dt_s);
    double rudCmd = 0.0;    

    // ---- Output & Log ----
    controlSystemPID->U_cmd.da = aileronCmd;
    controlSystemPID->U_cmd.de = elvCmd;
    controlSystemPID->U_cmd.dr = clamp(rudCmd, -25*deg2rad, 10*deg2rad);
    controlSystemPID->U_cmd.dthr[0] = throtCmd;
    controlSystemPID->U_cmd.dthr[1] = controlSystemPID->U_cmd.dthr[0];
        
    logger.data[LOG_U_CMD_DA] = controlSystemPID->U_cmd.da;
    logger.data[LOG_U_CMD_DE] = controlSystemPID->U_cmd.de;
    logger.data[LOG_U_CMD_DR] = controlSystemPID->U_cmd.dr;
    logger.data[LOG_U_CMD_DTHR1] = controlSystemPID->U_cmd.dthr[0];
    logger.data[LOG_U_CMD_DTHR2] = controlSystemPID->U_cmd.dthr[1];

}