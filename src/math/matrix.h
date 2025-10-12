#pragma once

#include <stdbool.h>

#include "src/math/vector.h"


Vector3 mat3_mult_vec3(const double M[3][3], const Vector3 v);

void mat3_mult(const double A[3][3], const double B[3][3], double C[3][3]);

void mat3_scale(const double m[3][3], double s, double mout[3][3]);

void mat3_transpose(const double m[3][3], double m_transpose[3][3]);

bool mat3_inv(const double M[3][3], double M_inv[3][3]);

void mat_identity(double* A, int n);

void mat_scale(double* A, int n, double scale);

void mat_transpose(double* A, int N);

void mat_mult(const double* A, const double* B, double* C, int N);

void mat_add(const double* A, const double* B, double* C, int N);

void matrixMultiply(const double* A, int n, int m, const double* B, int m2, int p, double* C);
