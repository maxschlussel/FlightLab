#include "src/controllers/PID.h"
#include "src/core/constants.h"
#include "src/guidance/guidance.h"
#include "src/math/utils.h"

#include "src/io/logger.h"

void computeFlightControlCmd(StateVector* X_est, GuidanceRefs* guidanceRefs, AircraftParams* acParams, ControlVector* U_cmd){
    // [0] Defne Useful Quantities
    Vector3 w_b = {X_est->p, X_est->q, X_est->r}; // Angular rates in body frame - omega_b
    Vector3 V_b = {X_est->u, X_est->v, X_est->w}; // Velocities in body frame - V_b
    Vector3 U_123 = {U_cmd->da, U_cmd->de, U_cmd->dr};  // Primary controls
    
    // [1] Compute Flight Quantities
    double velocity = vec3_norm(V_b);
    double Q = 0.5 * rho * pow(velocity, 2);  // Dynamic pressure
    double alpha = atan2(X_est->w, X_est->u);
    double beta = asin(X_est->v/velocity); //?????????

    double Ke = 0.003;
    U_cmd->de = Ke * (80 - velocity);
    U_cmd->de = clamp(U_cmd->de, -25*deg2rad, 10*deg2rad);
    
    double Kthr = -0.000075;
    U_cmd->dt[0] = 5.0*deg2rad + Kthr * (-1000 - X_est->z);
    
    U_cmd->dt[0] = clamp(U_cmd->dt[0], 0.5*deg2rad, 10*deg2rad);
    U_cmd->dt[1] = U_cmd->dt[0];

    logger.data[LOG_U_CMD_DA] = U_cmd->da;
    logger.data[LOG_U_CMD_DE] = U_cmd->de;
    logger.data[LOG_U_CMD_DR] = U_cmd->dr;
    logger.data[LOG_U_CMD_DTHR1] = U_cmd->dt[0];
    logger.data[LOG_U_CMD_DTHR2] = U_cmd->dt[1];

}