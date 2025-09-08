#pragma once

#include <math.h>
#include <stdbool.h>


/**
 * 3D vector representation.
*/
typedef struct {
    double x, y, z;
} Vector3;


Vector3 vec3_add(Vector3 a, Vector3 b);

Vector3 vec3_sub(Vector3 a, Vector3 b);

Vector3 vec3_scale(Vector3 a, double s);

double vec3_dot(Vector3 a, Vector3 b);

Vector3 vec3_mult(Vector3 a, Vector3 b);

Vector3 vec3_cross(Vector3 a, Vector3 b) ;

double vec3_norm(Vector3 v) ;
