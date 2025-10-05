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

#define N_EKF 18


typedef struct {
    int initialized;
    double x[N_EKF];          /* state */
    double P[N_EKF][N_EKF];   /* covariance */
    double Q[N_EKF][N_EKF];   /* process-noise (continuous -> discretized by dt) */
} EKF;

/* Singleton filter instance used by estimateStateEKF */
static EKF ekf_global;

/* ---------- Model and numeric jacobians ---------- */
/* continuous-time state derivative f(x) (xdot) */
static void f_state_dot(const double *x, double *xdot){
    /* state indices:
       0:u,1:v,2:w
       3:p,4:q,5:r
       6:phi,7:theta,8:psi
       9:x,10:y,11:z
       12..14 gyro bias
       15..17 accel bias
    */
    double u = x[0], v = x[1], w = x[2];
    double p = x[3], q = x[4], r = x[5];
    double phi = x[6], theta = x[7], psi = x[8];

    /* 1) body-velocity derivatives (kinematic coupling due to rotation)
       (neglecting external forces; in a better model you'd include forces/mass)
         u_dot = r*v - q*w
         v_dot = p*w - r*u
         w_dot = q*u - p*v
    */
    xdot[0] = r*v - q*w;
    xdot[1] = p*w - r*u;
    xdot[2] = q*u - p*v;

    /* angular rates derivatives - assume near-constant angular rates (random walk) */
    xdot[3] = 0.0;
    xdot[4] = 0.0;
    xdot[5] = 0.0;

    /* Euler rates from body rates: [phi_dot;theta_dot;psi_dot] = E * [p;q;r] */
    double cphi = cos(phi), sphi = sin(phi);
    double cth = cos(theta), sth = sin(theta);
    double tth = tan(theta);
    /* matrix E */
    double E[3][3] = {
        {1.0, sphi*tth, cphi*tth},
        {0.0, cphi,     -sphi},
        {0.0, sphi/cth, cphi/cth}
    };
    xdot[6] = E[0][0]*p + E[0][1]*q + E[0][2]*r;
    xdot[7] = E[1][0]*p + E[1][1]*q + E[1][2]*r;
    xdot[8] = E[2][0]*p + E[2][1]*q + E[2][2]*r;

    /* positions: body->NED velocity mapping */
    double R_e2b[3][3];
    getRotationMatrix(phi, theta, psi, R_e2b);
    /* R_b2n = transpose(R_e2b) */
    double R_b2n[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) R_b2n[i][j] = R_e2b[j][i];

    Vector3 v_body = {u,v,w};
    Vector3 v_ned = mat3_mult_vec3(R_b2n, v_body);
    xdot[9]  = v_ned.x;
    xdot[10] = v_ned.y;
    xdot[11] = v_ned.z;

    /* biases assumed random-walk with zero mean derivative */
    xdot[12] = 0.0; xdot[13] = 0.0; xdot[14] = 0.0;
    xdot[15] = 0.0; xdot[16] = 0.0; xdot[17] = 0.0;
}

/* Numeric Jacobian df/dx (n x n) using central differences */
static void numeric_jacobian_f(const double *x, double J[N_EKF][N_EKF]){
    double fplus[N_EKF], fminus[N_EKF];
    double xpert[N_EKF];
    for(int j=0;j<N_EKF;j++){
        /* adaptive eps */
        double eps = 1e-6 * (1.0 + fabs(x[j]));
        memcpy(xpert, x, sizeof(double)*N_EKF);
        xpert[j] = x[j] + eps;
        f_state_dot(xpert, fplus);
        memcpy(xpert, x, sizeof(double)*N_EKF);
        xpert[j] = x[j] - eps;
        f_state_dot(xpert, fminus);
        for(int i=0;i<N_EKF;i++){
            J[i][j] = (fplus[i] - fminus[i]) / (2.0 * eps);
        }
    }
}

/* Numeric jacobian of a scalar measurement h(x): returns row vector H[0..n-1] (dh/dx_j) */
static void numeric_jacobian_h_scalar(double (*hfun)(const double*), const double *x, double H_out[N_EKF]){
    double xplus[N_EKF], xminus[N_EKF];
    for(int j=0;j<N_EKF;j++){
        double eps = 1e-6 * (1.0 + fabs(x[j]));
        memcpy(xplus, x, sizeof(double)*N_EKF);
        memcpy(xminus, x, sizeof(double)*N_EKF);
        xplus[j]  = x[j] + eps;
        xminus[j] = x[j] - eps;
        double hp = hfun(xplus);
        double hm = hfun(xminus);
        H_out[j] = (hp - hm) / (2.0 * eps);
    }
}

/* ---------- Measurement prediction functions (scalar) ---------- */

/* gyro: z = p + gyro_bias_x */
static double h_gyro_x(const double *x){ return x[3] + x[12]; }
static double h_gyro_y(const double *x){ return x[4] + x[13]; }
static double h_gyro_z(const double *x){ return x[5] + x[14]; }

/* GPS pos: x,y,z in NED */
static double h_gps_x(const double *x){ return x[9]; }
static double h_gps_y(const double *x){ return x[10]; }
/* GPS z (NED down): return x[11] */
static double h_gps_z(const double *x){ return x[11]; }

/* Altimeter (assume altimeter gives altitude above mean sea level positive UP).
   Our state z is NED down positive, so altitude = -z */
static double h_altimeter(const double *x){ return -x[11]; }

/* Magnetometer / compass: assume preprocessed heading measurement (rad) */
static double h_mag_psi(const double *x){ return x[8]; }

/* Pitot/airspeed: ||v_body|| */
static double h_airspeed(const double *x){ return sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]); }

/* Accelerometer predicted measurement (body-frame): approx = -g_b + accel_bias
   where g_b = R_e2b * [0,0,g]^T, thus predicted accel = -g_b + bias.
   (This assumes negligible translational acceleration; it's an approximation.) */
static double h_accel_x(const double *x){
    double phi = x[6], theta = x[7], psi = x[8];
    double R[3][3]; getRotationMatrix(phi,theta,psi,R);
    Vector3 g_ned = {0.0, 0.0, g};
    Vector3 g_b = mat3_mult_vec3(R, g_ned); /* g in body */
    return -g_b.x + x[15];
}
static double h_accel_y(const double *x){
    double phi = x[6], theta = x[7], psi = x[8];
    double R[3][3]; getRotationMatrix(phi,theta,psi,R);
    Vector3 g_ned = {0.0, 0.0, g};
    Vector3 g_b = mat3_mult_vec3(R, g_ned); /* g in body */
    return -g_b.y + x[16];
}
static double h_accel_z(const double *x){
    double phi = x[6], theta = x[7], psi = x[8];
    double R[3][3]; getRotationMatrix(phi,theta,psi,R);
    Vector3 g_ned = {0.0, 0.0, g};
    Vector3 g_b = mat3_mult_vec3(R, g_ned); /* g in body */
    return -g_b.z + x[17];
}

/* ---------- EKF predict & scalar-update functions ---------- */

static void ekf_init_if_needed(void){
    if(ekf_global.initialized) return;
    ekf_global.initialized = 1;

    /* default state */
    for(int i=0;i<N_EKF;i++) ekf_global.x[i] = 0.0;
    /* set small initial forward velocity to avoid singularities */
    ekf_global.x[0] = 10.0; /* u ~ 10 m/s as a guess */

    /* covariance: initialize large for poorly-known states */
    for(int i=0;i<N_EKF;i++) for(int j=0;j<N_EKF;j++) ekf_global.P[i][j] = 0.0;
    for(int i=0;i<12;i++) ekf_global.P[i][i] = 1.0;      /* moderate uncertainty */
    ekf_global.P[9][9]  = ekf_global.P[10][10] = ekf_global.P[11][11] = 100.0; /* position big uncertainty */
    /* biases high uncertainty */
    ekf_global.P[12][12] = ekf_global.P[13][13] = ekf_global.P[14][14] = 0.1;
    ekf_global.P[15][15] = ekf_global.P[16][16] = ekf_global.P[17][17] = 0.1;

    /* process noise Q (continuous). Tune these values for your sim. */
    for(int i=0;i<N_EKF;i++) for(int j=0;j<N_EKF;j++) ekf_global.Q[i][j] = 0.0;
    /* velocity process noise */
    ekf_global.Q[0][0] = ekf_global.Q[1][1] = ekf_global.Q[2][2] = 0.5; /* (m/s^2)^2 */
    /* angular-rate noise */
    ekf_global.Q[3][3] = ekf_global.Q[4][4] = ekf_global.Q[5][5] = 0.01;
    /* attitude noise */
    ekf_global.Q[6][6] = ekf_global.Q[7][7] = ekf_global.Q[8][8] = 1e-4;
    /* position process noise (integration drift) */
    ekf_global.Q[9][9] = ekf_global.Q[10][10] = ekf_global.Q[11][11] = 0.1;
    /* bias random walk */
    ekf_global.Q[12][12] = ekf_global.Q[13][13] = ekf_global.Q[14][14] = 1e-6;
    ekf_global.Q[15][15] = ekf_global.Q[16][16] = ekf_global.Q[17][17] = 1e-6;
}

/* Predict step: x <- x + dt * f(x)
   P <- F * P * F^T + Qd (where F = I + dt * df/dx, Qd = Q*dt) */
static void ekf_predict(double dt){
    double xold[N_EKF];
    memcpy(xold, ekf_global.x, sizeof(xold));

    /* compute xdot = f(x) and advance */
    double xdot[N_EKF];
    f_state_dot(xold, xdot);
    for(int i=0;i<N_EKF;i++) ekf_global.x[i] = xold[i] + dt * xdot[i];

    /* numeric Jacobian df/dx */
    double J[N_EKF][N_EKF];
    numeric_jacobian_f(xold, J);

    /* F = I + dt * J */
    double F[N_EKF][N_EKF];
    for(int i=0;i<N_EKF;i++) for(int j=0;j<N_EKF;j++)
        F[i][j] = ((i==j)?1.0:0.0) + dt * J[i][j];

    /* P = F * P * F^T + Q * dt (simple discretization) */
    double tmp[N_EKF][N_EKF];
    memset(tmp, 0, sizeof(tmp));
    for(int i=0;i<N_EKF;i++){
        for(int k=0;k<N_EKF;k++){
            double fik = F[i][k];
            for(int j=0;j<N_EKF;j++){
                tmp[i][j] += fik * ekf_global.P[k][j];
            }
        }
    }
    double Pnew[N_EKF][N_EKF];
    memset(Pnew, 0, sizeof(Pnew));
    for(int i=0;i<N_EKF;i++){
        for(int j=0;j<N_EKF;j++){
            for(int k=0;k<N_EKF;k++){
                Pnew[i][j] += tmp[i][k] * F[j][k]; /* note F^T access */
            }
            /* add process noise discretized */
            Pnew[i][j] += ekf_global.Q[i][j] * dt;
        }
    }
    /* copy back and symmetrize */
    for(int i=0;i<N_EKF;i++) for(int j=0;j<N_EKF;j++) ekf_global.P[i][j] = 0.5*(Pnew[i][j] + Pnew[j][i]);
}

/* Scalar measurement update:
   z: measurement scalar
   hfun: pointer to scalar measurement function h(x)
   R: measurement variance (sigma^2)
*/
static void ekf_update_scalar(double z, double (*hfun)(const double*), double R){
    /* predicted measurement */
    double zpred = hfun(ekf_global.x);

    /* compute jacobian H (1xN) numerically */
    double H[N_EKF];
    numeric_jacobian_h_scalar(hfun, ekf_global.x, H);

    /* temp = P * H^T (n-vector) */
    double temp[N_EKF];
    for(int i=0;i<N_EKF;i++){
        temp[i] = 0.0;
        for(int j=0;j<N_EKF;j++) temp[i] += ekf_global.P[i][j] * H[j];
    }

    /* S = H * P * H^T + R = dot(H, temp) + R */
    double S = R;
    for(int j=0;j<N_EKF;j++) S += H[j] * temp[j];
    if(S <= 1e-12) S = 1e-12; /* guard */

    /* Kalman gain K = temp / S */
    double K[N_EKF];
    for(int i=0;i<N_EKF;i++) K[i] = temp[i] / S;

    /* innovation */
    double y = z - zpred;

    /* state update: x = x + K * y */
    for(int i=0;i<N_EKF;i++) ekf_global.x[i] += K[i] * y;

    /* covariance update: P = P - K * (H * P)  where (H * P) is 1xN vector HP[j] = sum_i H[i]*P[i][j] */
    double HP[N_EKF];
    for(int j=0;j<N_EKF;j++){
        HP[j] = 0.0;
        for(int i=0;i<N_EKF;i++) HP[j] += H[i] * ekf_global.P[i][j];
    }
    for(int i=0;i<N_EKF;i++){
        for(int j=0;j<N_EKF;j++){
            ekf_global.P[i][j] -= K[i] * HP[j];
        }
    }
    /* symmetrize to counter numerical drift */
    for(int i=0;i<N_EKF;i++) for(int j=0;j<N_EKF;j++)
        ekf_global.P[i][j] = 0.5 * (ekf_global.P[i][j] + ekf_global.P[j][i]);
}

/* Convert internal ekf state -> user StateVector */
static void ekf_to_StateVector(const double *x_in, StateVector *Xout){
    Xout->u = x_in[0]; Xout->v = x_in[1]; Xout->w = x_in[2];
    Xout->p = x_in[3]; Xout->q = x_in[4]; Xout->r = x_in[5];
    Xout->phi = x_in[6]; Xout->theta = x_in[7]; Xout->psi = x_in[8];
    Xout->x = x_in[9]; Xout->y = x_in[10]; Xout->z = x_in[11];
}

/* ----------------- Main func ------------------ */

/* sensors: pointer to latest measurement struct
   dt: time step (s)
   X_est: output estimated StateVector (filled by function)
*/
void estimateStateEKF(const Sensors* sensors, StateVector* X_est, double dt){
    /* init */
    ekf_init_if_needed();

    /* Predict */
    ekf_predict(dt);

    /* Now sequentially apply available measurements (scalar updates).
       Use sensor sigmaNoise fields where available for R. We fall back
       to sensible defaults if not present. */

    /* 1) IMU gyro (p,q,r) */
    /* measurement variance */
    double gyro_var = sensors->imuSensor.gyro.sigmaNoise * sensors->imuSensor.gyro.sigmaNoise;
    if(gyro_var <= 0.0) gyro_var = 0.001; /* fallback */

    ekf_update_scalar(sensors->imuSensor.gyro.data.x, h_gyro_x, gyro_var);
    ekf_update_scalar(sensors->imuSensor.gyro.data.y, h_gyro_y, gyro_var);
    ekf_update_scalar(sensors->imuSensor.gyro.data.z, h_gyro_z, gyro_var);

    /* 2) IMU accel (three axes) - treat as gravity vector reference (approx) */
    double accel_var = sensors->imuSensor.accel.sigmaNoise * sensors->imuSensor.accel.sigmaNoise;
    if(accel_var <= 0.0) accel_var = 0.5;
    ekf_update_scalar(sensors->imuSensor.accel.data.x, h_accel_x, accel_var);
    ekf_update_scalar(sensors->imuSensor.accel.data.y, h_accel_y, accel_var);
    ekf_update_scalar(sensors->imuSensor.accel.data.z, h_accel_z, accel_var);

    /* 3) GPS (if has_fix) - assume sensors->gps.x/y/z are NED positions (m) */
    if(sensors->gps.gps_valid){
        double gps_pos_var = 25.0; /* 5m std -> variance 25. Tune using real GPS HDOP. */
        ekf_update_scalar(sensors->gps.pos.data.x, h_gps_x, gps_pos_var);
        ekf_update_scalar(sensors->gps.pos.data.y, h_gps_y, gps_pos_var);
        ekf_update_scalar(sensors->gps.pos.data.z, h_gps_z, gps_pos_var);
    }

    /* 4) Altimeter */
    double alt_var = 4.0; /* 2 m std */
    ekf_update_scalar(sensors->altimeterSensor.alt, h_altimeter, alt_var);

    /* 5) Pitot (airspeed) */
    double pitot_var = 1.0; /* 1 m/s std */
    ekf_update_scalar(sensors->pitotTube.vel, h_airspeed, pitot_var);

    /* 6) Magnetometer (heading) */
    double mag_var = 0.05 * 0.05; /* ~0.05 rad std */
    double heading = computeHeadingFromMag(&(sensors->mag.data), X_est->phi, X_est->theta);
    ekf_update_scalar(heading, h_mag_psi, mag_var);

    /* Keep angles normalized */
    ekf_global.x[6] = wrapAngle(ekf_global.x[6], -pi, pi);
    ekf_global.x[7] = wrapAngle(ekf_global.x[7], -pi, pi);
    ekf_global.x[8] = wrapAngle(ekf_global.x[8], -pi, pi);

    /* Copy back to user's StateVector */
    ekf_to_StateVector(ekf_global.x, X_est);
}

/* Optional helper to reset EKF (if you restart sim). */
void ekf_reset(void){
    ekf_global.initialized = 0;
}
