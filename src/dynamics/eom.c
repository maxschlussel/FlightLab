#include <math.h>

#include "src/core/constants.h"
#include "src/dynamics/eom.h"
#include "src/math/dcm.h"
#include "src/math/matrix.h"
#include "src/math/quat.h"

#include "src/io/logger.h"


/**
 * @brief Computes the time derivatives of the state vector (6-DOF EOM).
 *
 * This function evaluates the rigid-body equations of motion for a fixed-wing aircraft.
 * It calculates the derivatives of translational velocities, angular rates,
 * Euler angles, and inertial positions, given the current state, aircraft properties,
 * and forces/moments.
 *
 * @param[in]  X            Pointer to current StateVector (u, v, w, p, q, r, phi, theta, psi, x, y, z).
 * @param[in]  acParams     Pointer to aircraft struct containing mass and inertial properties of the
 *                          aircraft.
 * @param[in]  F            Pointer to Vector3 containing the forces on the aircraft [N].
 * @param[in]  M            Pointer to Vector3 containing the moments on the aircraft [N-m].
 * @param[out] Xdot         Array of size 12 containing the state derivatives:
 *                          - [0]  u_dot   : forward velocity derivative (m/s^2)
 *                          - [1]  v_dot   : lateral velocity derivative (m/s^2)
 *                          - [2]  w_dot   : vertical velocity derivative (m/s^2)
 *                          - [3]  p_dot   : roll rate derivative (rad/s^2)
 *                          - [4]  q_dot   : pitch rate derivative (rad/s^2)
 *                          - [5]  r_dot   : yaw rate derivative (rad/s^2)
 *                          - [6]  phi_dot   : roll angle rate (rad/s)
 *                          - [7]  theta_dot : pitch angle rate (rad/s)
 *                          - [8]  psi_dot   : yaw angle rate (rad/s)
 *                          - [9]  x_dot   : inertial x-position derivative (m/s)
 *                          - [10] y_dot   : inertial y-position derivative (m/s)
 *                          - [11] z_dot   : inertial z-position derivative (m/s)
 */
void computeStateDerivative(const StateVector* X, const AircraftParams* acParams, const Vector3* F, const Vector3* M, double* Xdot){

    // [0] Define useful params
    Vector3 w_b = {X->p, X->q, X->r}; // Angular rates in body frame - omega_b
    Vector3 V_b = {X->u, X->v, X->w}; // Velocities in body frame - V_b
    double phi = X->phi, theta = X->theta, psi = X->psi;

    // [1] Compute translationsal EOM:
    // ---- Vdot_b = 1/m * F - omega_b X V_b ----
    Vector3 Vdot_b = vec3_sub(vec3_scale(*F, 1/acParams->mass), vec3_cross(w_b, V_b));

    // [2] Compute rotational EOM:
    double I_inv[3][3];
    mat3_inv(acParams->I, I_inv);
    
    // ---- wdot_b = I_inv * (M - w_b X I*w_b) ----
    Vector3 wdot_b = mat3_mult_vec3(I_inv, vec3_sub(*M, vec3_cross(w_b, mat3_mult_vec3(acParams->I, w_b))));

    // [3] Compute Euler angle kinematics
    double H[3][3] = {
        {1, sin(phi)*tan(theta), cos(phi)*tan(theta)},
        {0,            cos(phi),           -sin(phi)},
        {0, sin(phi)/cos(theta), cos(phi)/cos(theta)}
    };
    Vector3 euler_dot = mat3_mult_vec3(H, w_b);

    // [4] Compute position kinematics
    double R_e2b[3][3];  // Earth 2 body dcm
    getRotationMatrix(phi, theta, psi, R_e2b);
    double R_b2e[3][3];
    mat3_transpose(R_e2b, R_b2e);
    Vector3 dPos = mat3_mult_vec3(R_b2e, V_b);
    
    // [5] Fill derivatives vector
    Xdot[0] = Vdot_b.x;
    Xdot[1] = Vdot_b.y;
    Xdot[2] = Vdot_b.z;
    Xdot[3] = wdot_b.x;
    Xdot[4] = wdot_b.y;
    Xdot[5] = wdot_b.z;
    Xdot[6] = euler_dot.x;
    Xdot[7] = euler_dot.y;
    Xdot[8] = euler_dot.z;
    Xdot[9]  = dPos.x;
    Xdot[10] = dPos.y;
    Xdot[11] = dPos.z;

    logger.data[LOG_XDOT_U] = Xdot[0];
    logger.data[LOG_XDOT_V] = Xdot[1];
    logger.data[LOG_XDOT_W] = Xdot[2];
    logger.data[LOG_XDOT_P] = Xdot[3];
    logger.data[LOG_XDOT_Q] = Xdot[4];
    logger.data[LOG_XDOT_R] = Xdot[5];
    logger.data[LOG_XDOT_PHI]   = Xdot[6];
    logger.data[LOG_XDOT_THETA] = Xdot[7];
    logger.data[LOG_XDOT_PSI]   = Xdot[8];
    logger.data[LOG_XDOT_X] = Xdot[9];
    logger.data[LOG_XDOT_Y] = Xdot[10];
    logger.data[LOG_XDOT_Z] = Xdot[11];
}
