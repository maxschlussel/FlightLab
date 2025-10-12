#include "src/math/matrix.h"

/** Multiply a 3x3 martrix with a Vector3. */
Vector3 mat3_mult_vec3(const double M[3][3], const Vector3 v) {
    Vector3 vout;
    vout.x = M[0][0]*v.x + M[0][1]*v.y + M[0][2]*v.z;
    vout.y = M[1][0]*v.x + M[1][1]*v.y + M[1][2]*v.z;
    vout.z = M[2][0]*v.x + M[2][1]*v.y + M[2][2]*v.z;
    return vout;
}


/** Multiply a 3x3 martrix with a 3x3 martrix. */
void mat3_mult(const double A[3][3], const double B[3][3], double C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


/** Scale a 3x3 matrix by s. */
void mat3_scale(const double m[3][3], double s, double mout[3][3]){
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            mout[i][j] = m[i][j] * s;
        }
    }
}


/** Transpose a 3x3 matrix. */
void mat3_transpose(const double m[3][3], double m_transpose[3][3]){
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m_transpose[i][j] = m[j][i];
}


/** Compute inverse of a 3x3 matrix. Returns False if does not exist. */
bool mat3_inv(const double M[3][3], double M_inv[3][3]){
    double det =
          M[0][0] * (M[1][1]*M[2][2] - M[1][2]*M[2][1])
        - M[0][1] * (M[1][0]*M[2][2] - M[1][2]*M[2][0])
        + M[0][2] * (M[1][0]*M[2][1] - M[1][1]*M[2][0]);

    if (det == 0.0) {
        return false; // singular matrix, no inverse
    }

    double invDet = 1.0 / det;

    M_inv[0][0] =  (M[1][1]*M[2][2] - M[1][2]*M[2][1]) * invDet;
    M_inv[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) * invDet;
    M_inv[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * invDet;

    M_inv[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]) * invDet;
    M_inv[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * invDet;
    M_inv[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) * invDet;

    M_inv[2][0] =  (M[1][0]*M[2][1] - M[1][1]*M[2][0]) * invDet;
    M_inv[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) * invDet;
    M_inv[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * invDet;

    return true;  
}


/** Create an n x n identity matrix. */
void mat_identity(double* A, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            int index = i * n + j;
            A[index] = (i == j) ? 1.0 : 0.0;        }
        }
    }
    

/** Scale an n x n matrix. */
void mat_scale(double* A, int n, double scale) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            int index = i * n + j;
            A[index] *= scale;
        }
    }
}


/** Transpose an n x n matrix. */
void mat_transpose(double* A, int N) {
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            int index_ij = i * N + j;
            int index_ji = j * N + i;
            
            double temp = A[index_ij];
            
            A[index_ij] = A[index_ji];
            A[index_ji] = temp;
        }
    }
}


/** Multiply two n x n matricies. */
void mat_mult(const double* A, const double* B, double* C, int N) {
    // C[i][j] = sum(A[i][k] * B[k][j])
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            
            double sum = 0.0;
            
            for (int k = 0; k < N; k++) {
                int index_A = i * N + k;
                int index_B = k * N + j;
                
                sum += A[index_A] * B[index_B];
            }
            
            C[i * N + j] = sum;
        }
    }
}


/** Add two n x n matricies. */
void mat_add(const double* A, const double* B, double* C, int N) {
    for (int i = 0; i < N * N; i++) {
        C[i] = A[i] + B[i];
    }
}


// Matrix multiplication: C = A * B
// A: n x m, B: m x p, C: n x p
void matrixMultiply(const double* A, int n, int m, const double* B, int m2, int p, double* C) {
    if (m != m2) {
        // Dimension mismatch
        return;
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < p; j++) {
            double sum = 0.0;
            for (int k = 0; k < m; k++) {
                sum += A[i * m + k] * B[k * p + j];
            }
            C[i * p + j] = sum;
        }
    }
}
