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
