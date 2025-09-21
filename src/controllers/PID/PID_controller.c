#include "src/controllers/PID/PID_controller.h"
#include "src/controllers/PID/PID.h"
#include "src/math/utils.h"

#include "src/io/logger.h"


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
    
    // aileronCmd = 0.0;
    // elvCmd = -10.2*deg2rad;
    // rudCmd = 0.0;
    // throtCmd = 4.7*deg2rad;

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