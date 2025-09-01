#include <math.h>

#include "src/dynamics/aerodynamics.h"
#include "src/core/constants.h"
#include "src/math/dcm.h"

/**
 * Goal: compute aerodynamic forces and moments
 * 
 * 1. Compute flight quantities (alpha, beta, Q, etc.)
 * 2. Calculate body forces
 *      2.a Calculate CL, Cd, Cy
 *      2.b Rotate from stability frame to wind frame
 *      2.c Calculate Fx_b, Fy_b, Fz_b
 * 3. Calculate body moments
 *      2.a Calculate Cl, Cm, Cn
 *      2.b Rotate from stability frame to wind frame
 *      2.c Calculate Mx_b, My_b, Mz_b
 */
void computeAerodynamicForces(StateVector* X, ControlVector* U, AircraftParams* acParams, Vector3* F, Vector3* M){
    // saturateControls();

    // [0] Defne Useful Quantities
    Vector3 w_b = {X->p, X->q, X->r}; // Angular rates in body frame - omega_b
    Vector3 V_b = {X->u, X->v, X->w}; // Velocities in body frame - V_b
    Vector3 U_123 = {U->da, U->de, U->dr};  // Primary controls
    
    // [1] Compute Flight Quantities
    double velocity = vec3_norm(V_b);
    double Q = 0.5 * rho * pow(velocity, 2);  // Dynamic pressure
    double alpha = atan2(X->w, X->u);
    double beta = asin(X->v/velocity); //?????????

    // [2] Compute Aerodynamic Force
    double CL = computeCL(acParams, U, alpha);
    double Cd = computeCd(alpha);
    double Cy = computeCy(U, beta);
    Vector3 CF_stab = {-Cd, Cy, -CL};

    double QSfactor = Q * acParams->S;

    Vector3 F_stab = vec3_scale(CF_stab, QSfactor);

    // // Rotate from Fs to wind frame [-Cd, Cy, -Cl]
    // double R_stab_to_wind[3][3] = {
    //     { cos(beta), sin(beta), 0},
    //     {-sin(beta), cos(beta), 0},
    //     {         0,         0, 1}
    // };
    
    // double Cd_wind  = R_stab_to_wind[0][0]*coef_stability[0] + R_stab_to_wind[0][1]*coef_stability[1] + R_stab_to_wind[0][2]*coef_stability[2];
    // double Cy_wind  = R_stab_to_wind[1][0]*coef_stability[0] + R_stab_to_wind[1][1]*coef_stability[1] + R_stab_to_wind[1][2]*coef_stability[2];
    // double Cl_wind  = R_stab_to_wind[2][0]*coef_stability[0] + R_stab_to_wind[2][1]*coef_stability[1] + R_stab_to_wind[2][2]*coef_stability[2];
    
    // double Cf_w[3] = {Cd_wind, Cy_wind, Cl_wind};

    // // Get Forces in Fb
    // double Faero_stab[3] = {
    //     -Cd_wind * Q * S,
    //         Cy_wind * Q * S,
    //     -Cl_wind * Q * S
    // };

    // Rotate to body frame
    double R_stab_to_body[3][3] = {};
    getRotationMatrix(0, alpha, 0, R_stab_to_body);

    Vector3 F_body = mat3_mult_vec3(R_stab_to_body, F_stab);

    Vector3 CM_aero = computeCM(acParams, alpha, beta, velocity, &w_b, &U_123);

    double QSCfactor = Q * acParams->S * acParams->chord;

    Vector3 M_ac_body = vec3_scale(CM_aero, QSCfactor);

    Vector3 M_cg_body = vec3_cross(F_body, acParams->r_ac2cg);

    Vector3 M_body = vec3_add(M_ac_body, M_cg_body);

    F->x = F_body.x;
    F->y = F_body.y;
    F->z = F_body.z;
    M->x = M_body.x;
    M->y = M_body.y;
    M->z = M_body.z;
}

double computeCL(AircraftParams* acParams, ControlVector* U, double alpha){
    double CL_wingbody = computeCL_wingbody(acParams, alpha);
    double CL_tail     = computeCL_tail(acParams, U, alpha);  
    return CL_wingbody + CL_tail;
}

double computeCL_wingbody(AircraftParams* acParams, double alpha){
    double CL_wingbody;
    if (alpha <= acParams->alphaNonlinear){
        CL_wingbody = acParams->slope_CL_Alpha * (alpha - acParams->alpha_L0);
    }
    else{
        CL_wingbody = acParams->a3 * pow(alpha, 3) + acParams->a2 * pow(alpha, 2) + 
            acParams->a1 * alpha + acParams->a0;
    }
    return CL_wingbody;
}

double computeEpsilonDownwash(AircraftParams* acParams, double alpha){
    return acParams->dEpsDa * (alpha - acParams->alpha_L0);  // Downwash
}

double computeCL_tail(AircraftParams* acParams, ControlVector* U, double alpha){
    double eps = computeEpsilonDownwash(acParams, alpha);

    double alpha_tail = alpha - eps + U->de;  // Omitted for simplicity
                                              // + 1.3 * X.q * acParams->l_t / velocity;

    return 3.1 * (acParams->S_tail / acParams->S) * alpha_tail;
}

double computeCd(double alpha){
    return 0.13 + 0.07 * pow((5.5 * alpha + 0.654), 2);
}

double computeCy(ControlVector* U, double beta){
    return -1.6 * beta + 0.24 * U->dr;
}

Vector3 computeCM(AircraftParams* acParams, double alpha, double beta, double velocity, Vector3* w_b, Vector3* U_123){
    double eps = computeEpsilonDownwash(acParams, alpha);

    double eta[3] = {
        -1.4 * beta,
        -0.59 - 3.1 * acParams->S_tail * acParams->l_t * (alpha - eps) / (acParams->S * acParams->chord),
        (1 - alpha * rad2deg / 15) * beta
    };

    double dcm_dx_temp[3][3] = {
        {-11, 0, 5},
        {0, -4.03 * acParams->S_tail * pow(acParams->l_t, 2)/ (acParams->S * pow(acParams->chord, 2)), 0},
        {1.7, 0, -11.5}
    };

    double dcm_dx[3][3];
    mat3_scale(dcm_dx_temp, acParams->chord/velocity, dcm_dx);

    Vector3 term2 = mat3_mult_vec3(dcm_dx, *w_b);
        
    double dcm_du[3][3] = {
        {-0.6, 0, 0.22},
        {0, -3.1 * acParams->S_tail * acParams->l_t / (acParams->S * acParams->chord), 0},
        {0, 0, -0.63}
    };

    Vector3 term3 = mat3_mult_vec3(dcm_du, *U_123);

    double Cl_ac = eta[0] + term2.x + term3.x;
    double Cm_ac = eta[1] + term2.y + term3.x;
    double Cn_ac = eta[2] + term2.z + term3.x;
    
    Vector3 CM = {Cl_ac, Cm_ac, Cn_ac};
    return CM;
}
