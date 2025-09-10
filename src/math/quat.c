#include <math.h>

#include "src/core/constants.h"
#include "src/math/quat.h"

//** Scale quaternion by s. */
void quat_scale(Quat* q, double s){
    q->w *= s;
    q->x *= s;
    q->y *= s;
    q->z *= s;
}


//**  Normalize quaternion. */
void quat_normalize(Quat* q) {
    double norm = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    quat_scale(q, 1/norm);
}


//**  Conjugate quaternion (equivalent to inverse if normalized). */
Quat quat_conjugate(Quat q) {
    return (Quat){q.w, -q.x, -q.y, -q.z};
}


//** Convert Euler Angles rotation (ZYX) to a quaternion. */
Quat eulerToQuat(double phi, double theta, double psi){
    double cosPhi2   = cos(phi/2), sinPhi2 = sin(phi/2);
    double cosTheta2 = cos(theta/2), sinTheta2 = sin(theta/2);
    double cosPsi2   = cos(psi/2), sinPsi2 = sin(psi/2);
    
    Quat q = {
        .w = (cosPhi2 * cosTheta2 * cosPsi2) + (sinPhi2 * sinTheta2 * sinPsi2),
        .x = (sinPhi2 * cosTheta2 * cosPsi2) - (cosPhi2 * sinTheta2 * sinPsi2),
        .y = (cosPhi2 * sinTheta2 * cosPsi2) + (sinPhi2 * cosTheta2 * sinPsi2),
        .z = (cosPhi2 * cosTheta2 * sinPsi2) - (sinPhi2 * sinTheta2 * cosPsi2)
    };
    return q;
}


//** Multiply two quaternions (q1 ⊗ q2). */
Quat quat_mult(Quat q1, Quat q2) {
    return (Quat){
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    };
}


//** Rotate Vector3 v by quaternion q: v' = q * v * q^-1 */
Vector3 quat_rotate_vec3(Quat q, Vector3 v) {
    Quat vq = {0.0, v.x, v.y, v.z};
    Quat q_inv = quat_conjugate(q);
    Quat result = quat_mult(quat_mult(q, vq), q_inv);
    return (Vector3){result.x, result.y, result.z};
}


//** Convert quaternion to Euler Angles (ZYX). */
void quatToEuler(Quat q, double* phi, double* theta, double* psi){
    // Phi
    double phiNumerator = 2 * (q.w * q.x + q.y * q.z);
    double phiDenominator = 1 - 2 * (q.x*q.x + q.y*q.y);
    *phi = atan2(phiNumerator, phiDenominator);

    // Theta
    double sinTheta = 2.0 * (q.w*q.y - q.z*q.x);
    if (fabs(sinTheta) >= 1.0)
        *theta = copysign(pi / 2.0, sinTheta);
    else
        *theta = asin(sinTheta);

    // Psi
    double psiNumerator = 2 * (q.w*q.z + q.x*q.y);
    double psiDenominator = 1 - 2 * (q.y*q.y + q.z*q.z);
    *psi = atan2(psiNumerator, psiDenominator);
}


//** Rotate quaternion by body rate vector omega (w) for time dt_s */
Quat quat_rotate(Quat q, Vector3 w, double dt_s){
    Quat qw = {0.0, w.x, w.y, w.z};
    Quat qdot = quat_mult(q, qw);
    quat_scale(&qdot, 0.5);

    Quat q_rotated = {
        q.w + qdot.w * dt_s,
        q.x + qdot.x * dt_s,
        q.y + qdot.y * dt_s,
        q.z + qdot.z * dt_s
    };
    quat_normalize(&q_rotated);

    return q_rotated;
}
