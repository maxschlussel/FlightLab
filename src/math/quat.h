#pragma once

#include "src/math/vector.h"


typedef struct {
    double w, x, y, z;
} Quat;

void quat_scale(Quat* q, double s);

void quat_normalize(Quat* q);

Quat quat_conjugate(Quat q);

Quat eulerToQuat(double phi, double theta, double psi);

Quat quat_mult(Quat q1, Quat q2);

Vector3 quat_rotate_vec3(Quat q, Vector3 v);

void quatToEuler(Quat q, double* phi, double* theta, double* psi);

Quat quat_rotate(Quat q, Vector3 w, double dt_s);
