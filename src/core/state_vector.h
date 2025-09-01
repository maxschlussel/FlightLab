#pragma once

/**
 * @brief Represents the complete aircraft state vector (12 states).
 * 
 * This structure stores the translational and rotational states of the 
 * aircraft in different reference frames. It is commonly used as the 
 * primary state container for simulations and control algorithms.
*/
typedef struct {
    double u, v, w;         // Velocity [body frame]
    double p, q, r;         // Angular rates [body frame]
    double phi, theta, psi; // Euler angles [NED frame]
    double x, y, z;         // Position [NED frame]
} StateVector;

StateVector loadInitState(void);
