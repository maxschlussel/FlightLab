#pragma once

/**
 * @brief Represents the complete aircraft state vector (12 states).
 * 
 * This structure stores the translational and rotational states of the 
 * aircraft in different reference frames. It is commonly used as the 
 * primary state container for simulations and control algorithms.
*/
typedef struct {
    double u, v, w;         // Velocity [X-Y-Z body frame]
    double p, q, r;         // Angular rates [roll-pitch-yaw rates body frame]
    double phi, theta, psi; // Euler angles [roll-pitch-yaw NED frame]
    double x, y, z;         // Position [NED frame]
} StateVector;
