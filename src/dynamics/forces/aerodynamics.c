#include <math.h>

#include "src/dynamics/forces/aerodynamics.h"
#include "src/core/constants.h"
#include "src/math/dcm.h"
#include "src/math/matrix.h"

#include "src/io/logger.h"


/**
 * @brief Computes the aerodynamic forces and moments on the aircraft.
 *
 * This function calculates the aerodynamic forces and moments acting on the
 * aircraft based on its current state, control inputs, and aircraft parameters.
 *
 * @param[in]  X         Pointer to the aircraft's true StateVector.
 * @param[in]  U         Pointer to the ControlVector containing commanded
 *                       control deflections.
 * @param[in]  acModel  Pointer to the AircraftModel struct with mass,
 *                       inertia, and aircraft parameters.
 * @param[out] F         Pointer to a Vector3 where the resulting aerodynamic
 *                       forces in the body frame will be stored.
 * @param[out] M         Pointer to a Vector3 where the resulting aerodynamic
 *                       moments in the body frame will be stored.
 */
 void computeAerodynamicForces(const StateVector* X, const Actuators* actuators, const AircraftModel* acModel, AeroData* aeroData){
    // [0] Defne Useful Quantities
    Vector3 w_b = {X->p, X->q, X->r}; // Angular rates in body frame - omega_b
    Vector3 V_b = {X->u, X->v, X->w}; // Velocities in body frame - V_b
    Vector3 U_123 = {actuators->aileronServo.pos, 
                     actuators->elevatorServo.pos, 
                     actuators->rudderServo.pos};  // Primary controls
    
    // [1] Compute Flight Quantities
    double velocity = vec3_norm(V_b);
    double q_inf = 0.5 * rho * pow(velocity, 2);  // Dynamic pressure
    double alpha = atan2(X->w, X->u);
    double beta = asin(X->v/velocity); //?????????

    // [2] Compute Aerodynamic Force
    double CL = computeCL(acModel, U_123.y, alpha);
    double CD = computeCd(alpha);
    double CY = computeCy(U_123.z, beta);
    Vector3 CF_stab = {-CD, CY, -CL};

    double QSfactor = q_inf * acModel->S;

    Vector3 F_stab = vec3_scale(CF_stab, QSfactor);

    double R_stab_to_body[3][3] = {};
    getRotationMatrix(0, alpha, 0, R_stab_to_body);

    Vector3 F_body = mat3_mult_vec3(R_stab_to_body, F_stab);  // Rotate to body frame

    // [3] Computer Aerodynamic Moment
    Vector3 CM_aero = computeCM(acModel, alpha, beta, velocity, &w_b, &U_123);

    double QSCfactor = q_inf * acModel->S * acModel->chord;

    Vector3 M_ac_body = vec3_scale(CM_aero, QSCfactor);

    Vector3 M_cg_body = vec3_cross(F_body, acModel->r_ac2cg);

    Vector3 M_body = vec3_add(M_ac_body, M_cg_body);

    // [4]] Return AeroData
    aeroData->F_aero_net = F_body;
    aeroData->M_aero_net = M_body;
    
    aeroData->CL = CL;
    aeroData->CD = CD;
    aeroData->CY = CY;
    aeroData->Cl = CM_aero.x;
    aeroData->Cm = CM_aero.y;
    aeroData->Cn = CM_aero.z;

    aeroData->Lift = F_stab.z;
    aeroData->Drag = F_stab.x;
    aeroData->SideForce = F_stab.y;
    aeroData->PitchingMoment = CM_aero.x;
    aeroData->RollingMoment = CM_aero.y;
    aeroData->YawingMoment = CM_aero.z;
    
    aeroData->q_inf = q_inf;
    aeroData->alpha = alpha;
    aeroData->beta = beta;
    aeroData->velocity = velocity;

    logger.data[LOG_QINF] = q_inf;
    logger.data[LOG_ALPHA] = alpha;
    logger.data[LOG_BETA] = beta;
    logger.data[LOG_AERO_COEF_CL] = CL;
    logger.data[LOG_AERO_COEF_CD] = CD;
    logger.data[LOG_AERO_COEF_CY] = CY;
    logger.data[LOG_AERO_COEF_Cl] = CM_aero.x;
    logger.data[LOG_AERO_COEF_Cm] = CM_aero.y;
    logger.data[LOG_AERO_COEF_Cn] = CM_aero.z;
    logger.data[LOG_FORCES_F_AERO_X] = F_body.x;
    logger.data[LOG_FORCES_F_AERO_Y] = F_body.y;
    logger.data[LOG_FORCES_F_AERO_Z] = F_body.z;
    logger.data[LOG_MOMENTS_M_AERO_X] = M_body.x;
    logger.data[LOG_MOMENTS_M_AERO_Y] = M_body.y;
    logger.data[LOG_MOMENTS_M_AERO_Z] = M_body.z;
}


/**
 * @brief Computes the total lift coefficient for the aircraft.
 *
 * @param[in] acModel  Pointer to the AircraftModel struct containing
 *                      aircraft properties.
 * @param[in] de        Elevator deflection in radians.
 * @param[in] alpha     The angle of attack in radians.
 *
 * @return The total lift coefficient.
 */
double computeCL(const AircraftModel* acModel, double de, double alpha){
    double CL_wingbody = computeCL_wingbody(acModel, alpha);
    double CL_tail     = computeCL_tail(acModel, de, alpha);  
    return CL_wingbody + CL_tail;
}


/**
 * @brief Computes the wing-body lift coefficient.
 *
 * This function calculates the lift coefficient for the wing and body
 * combination. It uses a linear model for small angles of attack and
 * switches to a third-order polynomial for larger angles, as defined
 * by the `alphaNonlinear` parameter.
 *
 * @param[in] acModel  Pointer to the AircraftModel struct.
 * @param[in] alpha     The angle of attack in radians.
 *
 * @return The wing-body lift coefficient.
 */
double computeCL_wingbody(const AircraftModel* acModel, double alpha){
    double CL_wingbody;
    if (alpha <= acModel->alphaNonlinear){
        CL_wingbody = acModel->slope_CL_Alpha * (alpha - acModel->alpha_L0);
    }
    else{
        CL_wingbody = acModel->a3 * pow(alpha, 3) + acModel->a2 * pow(alpha, 2) + 
            acModel->a1 * alpha + acModel->a0;
    }
    return CL_wingbody;
}


/**
 * @brief Computes the downwash angle at the tail.
 *
 * This function calculates the downwash angle at the tail as a linear
 * function of the angle of attack.
 *
 * @param[in] acModel  Pointer to the AircraftModel struct.
 * @param[in] alpha     The angle of attack in radians.
 *
 * @return The downwash angle in radians.
 */
double computeEpsilonDownwash(const AircraftModel* acModel, double alpha){
    return acModel->dEpsDa * (alpha - acModel->alpha_L0);  // Downwash
}


/**
 * @brief Computes the lift coefficient for the horizontal tail.
 *
 * @param[in] acModel  Pointer to the `AircraftModel` struct with aircraft properties.
 * @param[in] U         Pointer to the `ControlVector` with control surface deflections.
 * @param[in] alpha     The aircraft's angle of attack in radians.
 *
 * @return The lift coefficient of the tail.
 */
double computeCL_tail(const AircraftModel* acModel, double de, double alpha){
    double eps = computeEpsilonDownwash(acModel, alpha);

    double alpha_tail = alpha - eps + de;   // Omitted for simplicity
                                            // + 1.3 * X.q * acModel->l_t / velocity;

    return 3.1 * (acModel->S_tail / acModel->S) * alpha_tail;
}


/**
 * @brief Computes the drag coefficient for the aircraft.
 *
 * @param[in] alpha The angle of attack in radians.
 *
 * @return The drag coefficient.
 */
double computeCd(double alpha){
    return 0.13 + 0.07 * pow((5.5 * alpha + 0.654), 2);
}


/**
 * @brief Computes the side-force coefficient for the aircraft.
 *
 * @param[in] U         Pointer to the `ControlVector` struct.
 * @param[in] beta      The sideslip angle in radians.
 *
 * @return The side-force coefficient.
 */
double computeCy(double dr, double beta){
    return -1.6 * beta + 0.24 * dr;
}


/**
 * @brief Computes the aerodynamic moment coefficients (Cl, Cm, Cn).
 *
 * This function calculates the aerodynamic moment coefficients about the
 * aircraft's center of gravity in the body frame. It is based on a linearized 
 * model that depends on angle of attack, sideslip, angular rates, and
 * control surface deflections.
 *
 * @param[in] acModel      Pointer to the `AircraftModel` struct.
 * @param[in] alpha         The angle of attack in radians.
 * @param[in] beta          The sideslip angle in radians.
 * @param[in] velocity      The magnitude of the body-frame velocity.
 * @param[in] w_b           Pointer to a `Vector3` with body-frame angular rates.
 * @param[in] U_123         Pointer to a `Vector3` with primary control deflections.
 *
 * @return A `Vector3` containing the moment coefficients in the body frame:
 * - x: Roll moment coefficient Cl
 * - y: Pitch moment coefficient Cm
 * - z: Yaw moment coefficient Cn
 */
Vector3 computeCM(const AircraftModel* acModel, double alpha, double beta, double velocity, Vector3* w_b, Vector3* U_123){
    double eps = computeEpsilonDownwash(acModel, alpha);

    double eta[3] = {
        -1.4 * beta,
        -0.59 - 3.1 * acModel->S_tail * acModel->l_t * (alpha - eps) / (acModel->S * acModel->chord),
        (1 - alpha * rad2deg / 15) * beta
    };

    double dcm_dx_temp[3][3] = {
        {-11, 0, 5},
        {0, -4.03 * acModel->S_tail * pow(acModel->l_t, 2)/ (acModel->S * pow(acModel->chord, 2)), 0},
        {1.7, 0, -11.5}
    };

    double dcm_dx[3][3];
    mat3_scale(dcm_dx_temp, acModel->chord/velocity, dcm_dx);

    Vector3 term2 = mat3_mult_vec3(dcm_dx, *w_b);
        
    double dcm_du[3][3] = {
        {-0.6, 0, 0.22},
        {0, -3.1 * acModel->S_tail * acModel->l_t / (acModel->S * acModel->chord), 0},
        {0, 0, -0.63}
    };

    Vector3 term3 = mat3_mult_vec3(dcm_du, *U_123);

    double Cl_ac = eta[0] + term2.x + term3.x;
    double Cm_ac = eta[1] + term2.y + term3.y;
    double Cn_ac = eta[2] + term2.z + term3.z;
    
    Vector3 CM = {Cl_ac, Cm_ac, Cn_ac};
    return CM;
}
