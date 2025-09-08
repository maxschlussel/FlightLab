#include "src/math/vector.h"


/** Add two vectors. */
Vector3 vec3_add(Vector3 a, Vector3 b){
    return (Vector3){a.x + b.x, a.y + b.y, a.z + b.z};
}


/** Subtract two vectors. */
Vector3 vec3_sub(Vector3 a, Vector3 b){
    return (Vector3){a.x - b.x, a.y - b.y, a.z - b.z};
}


/** Scale a vector by s. */
Vector3 vec3_scale(Vector3 a, double s){
    return (Vector3){a.x * s, a.y * s, a.z * s};
}


/** Dot product of two vectors. */
double vec3_dot(Vector3 a, Vector3 b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}


/** Simple element wise multiply of two vectors. */
Vector3 vec3_mult(Vector3 a, Vector3 b){
    return (Vector3){a.x*b.x, a.y*b.y, a.z*b.z};
}


/** Cross product of two vectors. */
Vector3 vec3_cross(Vector3 a, Vector3 b) {
    return (Vector3){
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}


/** Magnitude (length) of a vector. */
double vec3_norm(Vector3 v) {
    return sqrt(vec3_dot(v, v));
}
