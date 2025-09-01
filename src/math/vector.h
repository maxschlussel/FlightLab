#pragma once

#include <math.h>
#include <stdbool.h>


/**
 * 3D vector representation.
*/
typedef struct {
    double x, y, z;
} Vector3;


/** Add two vectors. */
static inline Vector3 vec3_add(Vector3 a, Vector3 b){
    return (Vector3){a.x + b.x, a.y + b.y, a.z + b.z};
}


/** Subtract two vectors. */
static inline Vector3 vec3_sub(Vector3 a, Vector3 b){
    return (Vector3){a.x - b.x, a.y - b.y, a.z - b.z};
}


/** Scale a vector by s. */
static inline Vector3 vec3_scale(Vector3 a, double s){
    return (Vector3){a.x * s, a.y * s, a.z * s};
}


/** Dot product of two vectors. */
static inline double vec3_dot(Vector3 a, Vector3 b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}


/** Simple element wise multiply of two vectors. */
static inline Vector3 vec3_mult(Vector3 a, Vector3 b){
    return (Vector3){a.x*b.x, a.y*b.y, a.z*b.z};
}


/** Cross product of two vectors. */
static inline Vector3 vec3_cross(Vector3 a, Vector3 b) {
    return (Vector3){
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}


/** Magnitude (length) of a vector. */
static inline double vec3_norm(Vector3 v) {
    return sqrt(vec3_dot(v, v));
}


/** Multiply a 3x3 martrix with a Vector3. */
static inline Vector3 mat3_mult_vec3(const double M[3][3], Vector3 v) {
    Vector3 vout;
    vout.x = M[0][0]*v.x + M[0][1]*v.y + M[0][2]*v.z;
    vout.y = M[1][0]*v.x + M[1][1]*v.y + M[1][2]*v.z;
    vout.z = M[2][0]*v.x + M[2][1]*v.y + M[2][2]*v.z;
    return vout;
}


/** Multiply a 3x3 martrix with a 3x3 martrix. */
static inline void mat3_mult(const double A[3][3], const double B[3][3], double C[3][3]) {
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
static inline void mat3_scale(const double m[3][3], double s, double mout[3][3]){
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            mout[i][j] = m[i][j] * s;
        }
    }
}


/** Compute inverse of a 3x3 matrix. Returns False if does not exist. */
static inline bool mat3_inv(const double M[3][3], double M_inv[3][3]){
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
