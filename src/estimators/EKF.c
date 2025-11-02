#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "src/estimators/EKF.h"
#include "src/math/matrix.h"
#include "src/math/dcm.h"
#include "src/math/vector.h"
#include "src/math/utils.h"
#include "src/core/constants.h"
#include "src/estimators/simple_estimator.h"
#include "src/dynamics/eom.h"
#include "src/dynamics/forces.h"

#include "src/io/logger.h"

// Module in development...

EKF initEKF(const AircraftModel* acModel, const Actuators* actuators) {
    EKF ekf = {{0.0}};
    ekf.X[0] = 80.0;
    
    ekf.acModel = acModel;
    ekf.actuators  = actuators;

    return ekf;
}


void EKF_predictState(EKF* ekf, double dt) {
    
    StateVector X;
    array_to_statevec(ekf->X, &X);
    
    // [1] Compute f (state derivative)
    computeForcesAndMoments(&X, ekf->actuators, ekf->acModel, &(ekf->aeroData));

    computeStateDerivative(&X, ekf->acModel, &(ekf->aeroData.F_tot), &(ekf->aeroData.M_tot), ekf->Xdot);
    
    // [2] Propogate state (simple Euler integration)
    for (int i = 0; i < N_EKF_STATE; i++) {
        ekf->X[i] += ekf->Xdot[i] * dt;
    }
}

// F = I + (df/dx)*dt
void EKF_computeProcessJacobianF(EKF* ekf) {
    // [0] Define useful params
    double u = ekf->X[0], v = ekf->X[1], w = ekf->X[2];
    double p = ekf->X[3], q = ekf->X[4], r = ekf->X[5];
    double phi = ekf->X[6], theta = ekf->X[7], psi = ekf->X[8];

    double dt = ekf->dt;
    
    // Initialize F as identity matrix
    mat_identity(&ekf->F[0][0], N_EKF_STATE);

    // [1] Translational dynamics: Vdot_b = (1/m)*F - w_b x V_b
    // Partial derivatives of Vdot_b w.r.t. V_b and w_b
    // w_b x V_b = [q w - r v, 
    //              r u - p w, 
    //              p v - q u]
    // d(Vdot_b)/dV_b = -skew(w_b)
    // d(Vdot_b)/w_b = -skew(V_b)

    ekf->F[0][1] += dt * r;    // du_dot/dv
    ekf->F[0][2] += dt * (-q); // du_dot/dw
    ekf->F[0][4] += dt * (-w); // du_dot/dq
    ekf->F[0][5] += dt * v;    // du_dot/dr

    ekf->F[1][0] += dt * (-r); // dv_dot/du
    ekf->F[1][2] += dt * p;    // dv_dot/dw
    ekf->F[1][3] += dt * w;    // dv_dot/dp
    ekf->F[1][5] += dt * (-u); // dv_dot/dr
    
    ekf->F[2][0] += dt * q;    // dw_dot/du
    ekf->F[2][1] += dt * (-p); // dw_dot/dv
    ekf->F[2][3] += dt * (-v); // dw_dot/dp
    ekf->F[2][4] += dt * u;    // dw_dot/dq
    
    // [2] Rotational dynamics: wdot_b = I_inv * (M - w_b x I*w_b)
    double I_inv[3][3];
    mat3_inv(ekf->acModel->I, I_inv);
    double I[3][3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) I[i][j] = ekf->acModel->I[i][j];

    // Compute d(w_b x I*w_b)/dp, dq, dr
    double dCross_dp[3] = {
        q * I[2][0] - r * I[1][0],  // q*I_zx - r*I_yx
        -r * I[0][0] + p * I[2][0], // -r*I_xx + p*I_zx
        p * I[1][0] - q * I[0][0]   // p*I_yx - q*I_xx
    };
    double dCross_dq[3] = {
        q * I[2][1] - r * I[1][1] + (I[2][0] * p + I[2][1] * q + I[2][2] * r),
        -r * I[0][1] + p * I[2][1], // -r*I_xy + p*I_zy
        p * I[1][1] - q * I[0][1] - (I[0][0] * p + I[0][1] * q + I[0][2] * r)
    };
    double dCross_dr[3] = {
        q * I[2][2] - r * I[1][2] - (I[1][0] * p + I[1][1] * q + I[1][2] * r),
        -r * I[0][2] + p * I[2][2] + (I[0][0] * p + I[0][1] * q + I[0][2] * r),
        p * I[1][2] - q * I[0][2]
    };

    // Apply I_inv to get d(wdot_b)/dp, dq, dr
    for (int i = 0; i < 3; i++) {
        ekf->F[3][3] += dt * I_inv[i][0] * (-dCross_dp[0]) + dt * I_inv[i][1] * (-dCross_dp[1]) + dt * I_inv[i][2] * (-dCross_dp[2]); // dp_dot/dp
        ekf->F[4][3] += dt * I_inv[i][0] * (-dCross_dp[0]) + dt * I_inv[i][1] * (-dCross_dp[1]) + dt * I_inv[i][2] * (-dCross_dp[2]); // dq_dot/dp
        ekf->F[5][3] += dt * I_inv[i][0] * (-dCross_dp[0]) + dt * I_inv[i][1] * (-dCross_dp[1]) + dt * I_inv[i][2] * (-dCross_dp[2]); // dr_dot/dp

        ekf->F[3][4] += dt * I_inv[i][0] * (-dCross_dq[0]) + dt * I_inv[i][1] * (-dCross_dq[1]) + dt * I_inv[i][2] * (-dCross_dq[2]); // dp_dot/dq
        ekf->F[4][4] += dt * I_inv[i][0] * (-dCross_dq[0]) + dt * I_inv[i][1] * (-dCross_dq[1]) + dt * I_inv[i][2] * (-dCross_dq[2]); // dq_dot/dq
        ekf->F[5][4] += dt * I_inv[i][0] * (-dCross_dq[0]) + dt * I_inv[i][1] * (-dCross_dq[1]) + dt * I_inv[i][2] * (-dCross_dq[2]); // dr_dot/dq

        ekf->F[3][5] += dt * I_inv[i][0] * (-dCross_dr[0]) + dt * I_inv[i][1] * (-dCross_dr[1]) + dt * I_inv[i][2] * (-dCross_dr[2]); // dp_dot/dr
        ekf->F[4][5] += dt * I_inv[i][0] * (-dCross_dr[0]) + dt * I_inv[i][1] * (-dCross_dr[1]) + dt * I_inv[i][2] * (-dCross_dr[2]); // dq_dot/dr
        ekf->F[5][5] += dt * I_inv[i][0] * (-dCross_dr[0]) + dt * I_inv[i][1] * (-dCross_dr[1]) + dt * I_inv[i][2] * (-dCross_dr[2]); // dr_dot/dr
    }

    // [3] Euler angle kinematics: Phi_dot = H * w_b
    double tan_theta = tan(theta), sec_theta = 1.0 / cos(theta);
    ekf->F[6][3] += dt * 1.0;                    // dphi_dot/dp
    ekf->F[6][4] += dt * sin(phi) * tan_theta;   // dphi_dot/dq
    ekf->F[6][5] += dt * cos(phi) * tan_theta;   // dphi_dot/dr
    ekf->F[6][6] += dt * (q * cos(phi) * tan_theta - r * sin(phi) * tan_theta); // dphi_dot/dphi
    ekf->F[6][7] += dt * (q * sin(phi) + r * cos(phi)) * (sec_theta * sec_theta); // dphi_dot/dtheta

    ekf->F[7][4] += dt * cos(phi);               // dtheta_dot/dq
    ekf->F[7][5] += dt * (-sin(phi));            // dtheta_dot/dr
    ekf->F[7][6] += dt * (-q * sin(phi) - r * cos(phi)); // dtheta_dot/dphi

    ekf->F[8][4] += dt * sin(phi) * sec_theta;   // dpsi_dot/dq
    ekf->F[8][5] += dt * cos(phi) * sec_theta;   // dpsi_dot/dr
    ekf->F[8][6] += dt * (q * cos(phi) - r * sin(phi)) * sec_theta; // dpsi_dot/dphi
    ekf->F[8][7] += dt * (q * sin(phi) + r * cos(phi)) * (-sin(theta) * sec_theta * sec_theta); // dpsi_dot/dtheta


    // [4] Position kinematics: P_dot = R_b2e * V_b
    double R_b2e[3][3], R_e2b[3][3];
    getRotationMatrix(phi, theta, psi, R_e2b);
    mat3_transpose(R_e2b, R_b2e);

    ekf->F[9][0] += dt * R_b2e[0][0]; // dx_dot/du
    ekf->F[9][1] += dt * R_b2e[0][1]; // dx_dot/dv
    ekf->F[9][2] += dt * R_b2e[0][2]; // dx_dot/dw
    ekf->F[10][0] += dt * R_b2e[1][0]; // dy_dot/du
    ekf->F[10][1] += dt * R_b2e[1][1]; // dy_dot/dv
    ekf->F[10][2] += dt * R_b2e[1][2]; // dy_dot/dw
    ekf->F[11][0] += dt * R_b2e[2][0]; // dz_dot/du
    ekf->F[11][1] += dt * R_b2e[2][1]; // dz_dot/dv
    ekf->F[11][2] += dt * R_b2e[2][2]; // dz_dot/dw

    // Derivatives w.r.t. phi, theta, psi
    double dR_b2e_dphi[3][3], dR_b2e_dtheta[3][3], dR_b2e_dpsi[3][3];
    double dR_e2b_dphi[3][3] = {
        {0, 0, 0},
        {cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi), cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi), cos(phi) * cos(theta)},
        {-sin(phi) * sin(theta) * cos(psi) + cos(phi) * sin(psi), -sin(phi) * sin(theta) * sin(psi) - cos(phi) * cos(psi), -sin(phi) * cos(theta)}
    };
    double dR_e2b_dtheta[3][3] = {
        {-sin(theta) * cos(psi), -sin(theta) * sin(psi), -cos(theta)},
        {sin(phi) * cos(theta) * cos(psi), sin(phi) * cos(theta) * sin(psi), -sin(phi) * sin(theta)},
        {cos(phi) * cos(theta) * cos(psi), cos(phi) * cos(theta) * sin(psi), -cos(phi) * sin(theta)}
    };
    double dR_e2b_dpsi[3][3] = {
        {-cos(theta) * sin(psi), cos(theta) * cos(psi), 0},
        {-sin(phi) * sin(theta) * sin(psi) - cos(phi) * cos(psi), sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi), 0},
        {-cos(phi) * sin(theta) * sin(psi) + sin(phi) * cos(psi), cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi), 0}
    };
    mat3_transpose(dR_e2b_dphi, dR_b2e_dphi);
    mat3_transpose(dR_e2b_dtheta, dR_b2e_dtheta);
    mat3_transpose(dR_e2b_dpsi, dR_b2e_dpsi);

    Vector3 V_b = {u, v, w};
    Vector3 dP_dphi, dP_dtheta, dP_dpsi;
    dP_dphi = mat3_mult_vec3(dR_b2e_dphi, V_b);
    dP_dtheta = mat3_mult_vec3(dR_b2e_dtheta, V_b);
    dP_dpsi = mat3_mult_vec3(dR_b2e_dpsi, V_b);

    ekf->F[9][6] += dt * dP_dphi.x;   // dx_dot/dphi
    ekf->F[9][7] += dt * dP_dtheta.x; // dx_dot/dtheta
    ekf->F[9][8] += dt * dP_dpsi.x;   // dx_dot/dpsi
    ekf->F[10][6] += dt * dP_dphi.y;  // dy_dot/dphi
    ekf->F[10][7] += dt * dP_dtheta.y;// dy_dot/dtheta
    ekf->F[10][8] += dt * dP_dpsi.y;  // dy_dot/dpsi
    ekf->F[11][6] += dt * dP_dphi.z;  // dz_dot/dphi
    ekf->F[11][7] += dt * dP_dtheta.z;// dz_dot/dtheta
    ekf->F[11][8] += dt * dP_dpsi.z;  // dz_dot/dpsi
}


void EKF_PredictCovariance(EKF* ekf) {
    // P = F * P * F^T + Q
    double P_temp[N_EKF_STATE][N_EKF_STATE] = {0.0};
    
    // P_temp = F * P
    mat_mult(&ekf->F[0][0], &ekf->P[0][0], &P_temp[0][0], N_EKF_STATE);

    // P = P_temp * F^T + Q
    mat_mult(&P_temp[0][0], &ekf->F_T[0][0], &ekf->P[0][0], N_EKF_STATE);
    mat_add(&ekf->P[0][0], &ekf->Q[0][0], &ekf->P[0][0], N_EKF_STATE);
}


void EKF_commpute_h(EKF* ekf) {
    // [0] IMU: p, q, r
    ekf->Z_pred[0] = ekf->X[3]; // p
    ekf->Z_pred[1] = ekf->X[4]; // q
    ekf->Z_pred[2] = ekf->X[5]; // r

    // [1] IMU: body-frame acceleration
    ekf->Z_pred[3] = 0; // a_x
    ekf->Z_pred[4] = 0; // a_y
    ekf->Z_pred[5] = 0; // a_z

    // [2] GPS: x, y, z, dx, dy, dz
    ekf->Z_pred[6] = ekf->X[9];  // X
    ekf->Z_pred[7] = ekf->X[10]; // y
    ekf->Z_pred[8] = ekf->X[11]; // z
    ekf->Z_pred[9] = 0; //ekf->X[0];  // u
    ekf->Z_pred[10] = 0; //ekf->X[1]; // v
    ekf->Z_pred[11] = 0; //ekf->X[2]; // w

    // [4] Pitot: airspeed
    // Compute airspeed from u, v, w
    ekf->Z_pred[12] = sqrt(ekf->X[0]*ekf->X[0] + ekf->X[1]*ekf->X[1] + ekf->X[2]*ekf->X[2]);
    
    // [3] Magnetometer: psi
    ekf->Z_pred[13] = 0;


    // [5] Altimeter: altitude
    ekf->Z_pred[14] = -ekf->X[11]; // z
}


// Compute measurement residual: y = z - h(x)
void EKF_ComputeMeasurementResidual(EKF* ekf, Sensors* sensors) {
    double z[N_EKF_MEAS];
    sensors_to_array(sensors, z);

    for (int i = 0; i < N_EKF_MEAS; i++) {
        ekf->y[i] = z[i] - ekf->Z_pred[i];
    }
}


void EKF_computeMeasurmentJacobianH(EKF* ekf) {
    // Zero out H
    for (int i = 0; i < N_EKF_MEAS; i++) {
        for (int j = 0; j < N_EKF_STATE; j++) {
            ekf->H[i][j] = 0.0;
        }
    }

    // [0] IMU: p, q, r
    ekf->H[0][3] = 1.0; // h0 = p = x3
    ekf->H[1][4] = 1.0; // h1 = q = x4
    ekf->H[2][5] = 1.0; // h2 = r = x5

    // [1] IMU: a_x, a_y, a_z

    // [2] GPS: x, y, z, u, v, w
    ekf->H[6][9] = 1;  // h6 = x = x9
    ekf->H[7][10] = 1; // h7 = y = x10
    ekf->H[8][11] = 1; // h8 = z = x11
    ekf->H[9][0] = 1;  // h9 = u = x0
    ekf->H[10][1] = 1; // h10 = v = x1
    ekf->H[11][2] = 1; // h11 = w = x2

    // [3] Magnetometer: m_x, m_y, m_z

    // [4] Pitot: airspeed
    double u = ekf->X[0], v = ekf->X[1], w = ekf->X[2];
    double airspeed = sqrt(u*u + v*v + w*w);
    if (airspeed > 0) {
        ekf->H[12][0] = u / airspeed;
        ekf->H[12][1] = v / airspeed;
        ekf->H[12][2] = w / airspeed;
    }

    // [5] Altimeter: altitude
    ekf->H[13][11] = 1; // h16 = z = x11
}


void compute_kalman_gain(EKF* ekf) {
    // 2. Compute Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
    double S[N_EKF_MEAS][N_EKF_MEAS]; // Innovation covariance
    double K[N_EKF_STATE][N_EKF_MEAS]; // Kalman gain

    // S = H * P * H^T + R
    for (int i = 0; i < N_EKF_MEAS; i++) {
        for (int j = 0; j < N_EKF_MEAS; j++) {
            S[i][j] = ekf->R[i][j];
            for (int k = 0; k < N_EKF_STATE; k++) {
                for (int l = 0; l < N_EKF_STATE; l++) {
                    S[i][j] += ekf->H[i][k] * ekf->P[k][l] * ekf->H[j][l];
                }
            }
        }
    }

    // K = P * H^T * S_inv
    for (int i = 0; i < N_EKF_STATE; i++) {
        for (int j = 0; j < N_EKF_MEAS; j++) {
            K[i][j] = 0;
            for (int k = 0; k < N_EKF_STATE; k++) {
                for (int l = 0; l < N_EKF_MEAS; l++) {
                    K[i][j] += ekf->P[i][k] * ekf->H[j][k] * S[l][k];
                }
            }
        }
    }


    // 3. Update state: x = x + K * y
    for (int i = 0; i < N_EKF_STATE; i++) {
        for (int j = 0; j < N_EKF_MEAS; j++) {
            ekf->X[i] += K[i][j] * ekf->y[j];
        }
    }

    // 4. Update covariance: P = (I - K * H) * P
    double I_KH[N_EKF_STATE][N_EKF_STATE];
    for (int i = 0; i < N_EKF_STATE; i++) {
        for (int j = 0; j < N_EKF_STATE; j++) {
            I_KH[i][j] = (i == j) ? 1.0 : 0.0;
            for (int k = 0; k < N_EKF_MEAS; k++) {
                I_KH[i][j] -= K[i][k] * ekf->H[k][j];
            }
        }
    }
    double P_temp[N_EKF_STATE][N_EKF_STATE];
    for (int i = 0; i < N_EKF_STATE; i++) {
        for (int j = 0; j < N_EKF_STATE; j++) {
            P_temp[i][j] = 0;
            for (int k = 0; k < N_EKF_STATE; k++) {
                P_temp[i][j] += I_KH[i][k] * ekf->P[k][j];
            }
        }
    }
    for (int i = 0; i < N_EKF_STATE; i++) {
        for (int j = 0; j < N_EKF_STATE; j++) {
            ekf->P[i][j] = P_temp[i][j];
        }
    }

}


void estimateStateEKF(EKF* ekf, Sensors* sensors, double dt){
    // [1] Predict state: X_hat{k|k-1} = f(X{k-1|k-1}, u{k})
    EKF_predictState(ekf, dt);

    // [2] Compute process Jacobian: F = I + (df/dx)*dt
    // (Jacobian of state transition function f)
    EKF_computeProcessJacobianF(ekf);    

    // [3] Predict covariance: P_pred = F*P*F^T + Q
    EKF_PredictCovariance(ekf);

    // [4] Compute predicted measurment h(x)
    EKF_commpute_h(ekf);
    
    // [5] compute measurement residual innovation
    EKF_ComputeMeasurementResidual(ekf, sensors);

    // [6] Comupute the measurment Jacobian: H = dh/dx
    EKF_computeMeasurmentJacobianH(ekf);

    // [7] Compute innovation covariance: S = H*P_pred*H^T + R
    compute_kalman_gain(ekf);

    // [8] Compute Kalman gain: K = P_pred*H^T*S^-1
    
    // [9] Update state: x = x_pred + K*v    
    
    // [10] Update covariance: P = (I - K*H)*P_pred}

    logger.data[LOG_X_EST_U] = ekf->X[0];
    logger.data[LOG_X_EST_V] = ekf->X[1];
    logger.data[LOG_X_EST_W] = ekf->X[2];
    logger.data[LOG_X_EST_P] = ekf->X[3];
    logger.data[LOG_X_EST_Q] = ekf->X[4];
    logger.data[LOG_X_EST_R] = ekf->X[5];
    logger.data[LOG_X_EST_PHI]   = ekf->X[6];
    logger.data[LOG_X_EST_THETA] = ekf->X[7];
    logger.data[LOG_X_EST_PSI]   = ekf->X[8];
    logger.data[LOG_X_EST_X] = ekf->X[9];
    logger.data[LOG_X_EST_Y] = ekf->X[10];
    logger.data[LOG_X_EST_Z] = ekf->X[11];
}