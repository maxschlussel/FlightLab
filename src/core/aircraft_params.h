#pragma once

#include "src/math/vector.h"

/**
 * Basic (constant) aircraft parameters including geometry, mass properties,
 * and aerodynamic reference values required for flight dynamics simulation. 
 * Unit agnostic.
 */
typedef struct {
    char name[30];
    double mass;    // Total aircraft mass
    double I[3][3]; // Inertia matrix
    
    double chord;   // Mean aerodynamic chord
    double l_t;     // Tail moment arm (distance between aircraft cg and tail ac)
    double S;       // Wing reference planform area
    double S_tail;  // Tail reference planform area
    
    Vector3 cg;     // Positiong of center of gravity
    Vector3 ac;     // Position of aerodynamic center
    Vector3 engineOneThrustPoint; // Application point for engine 1 thrust vector
    Vector3 engineTwoThrustPoint; // Application point for engine 2 thrust vector

    double dEpsDa;      // Change in downwash W.R.T. alpha
    
    double alpha_L0;    // Zero lift angle of attack
    double slope_CL_Alpha;   // Slope of the linear region of CL vs Alpha
    double a3;  // Coefficient of alpha^3 for nonlinear region of CL vs Alpha
    double a2;  // Coefficient of alpha^2 for nonlinear region of CL vs Alpha
    double a1;  // Coefficient of alpha^1 for nonlinear region of CL vs Alpha
    double a0;  // Coefficient of alpha^0 for nonlinear region of CL vs Alpha
    double alphaNonlinear;  // Alpha where lift slope switches from linear to nonlinear

    Vector3 r_ac2cg;        // Distance from ac to cg
    Vector3 r_engineOne2cg;    // Distance from engine one thrust point to cg
    Vector3 r_engineTwo2cg;    // Distance from engine two thrust point to cg
} AircraftModel;
