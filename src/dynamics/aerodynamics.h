#pragma once

#include "src/core/aircraft_params.h"
#include "src/actuators/actuators.h"
#include "src/core/state_vector.h"
#include "src/math/vector.h"


/**
 * @brief Stores all calculated aerodynamic results (forces, moments, coefficients, and aero params)
 */
typedef struct {
    // --- Net Forces and Moments (body frame) ---
    Vector3 F_aero_net;     // Net aerodynamic forces
    Vector3 M_aero_net;     // Net aerodynamic moments
    
    Vector3 F_prop_net;     // Net propulsino forces
    Vector3 M_prop_net;     // Net propulsion moments

    Vector3 F_ground_net;   // Net ground forces
    Vector3 M_ground_net;   // Net ground moments
    
    Vector3 F_grav_net;     // Net gravity forces
    
    Vector3 F_tot;          // Total forces
    Vector3 M_tot;          // Total moments

    // --- Aerodynamic Coefficients (Normalized) ---
    double CL;              // Coefficient of Lift
    double CD;              // Coefficient of Drag
    double CY;              // Coefficient of sideforce
    double Cm;              // Coefficient of Pitching Moment (about body Y-axis)
    double Cl;              // Coefficient of Rolling Moment (about body X-axis)
    double Cn;              // Coefficient of Yawing Moment (about body Z-axis)
    
    // --- Intermediate Components ---
    double Lift;            // Total Lift Force
    double Drag;            // Total Drag Force
    double SideForce;       // Total Side Force
    // double Thrust;          // Total thrust Force
    double PitchingMoment;  // Pitching Moment component
    double RollingMoment;   // Rolling Moment component
    double YawingMoment;    // Yawing Moment component
    
    // --- Aerodynamic Parameters ---
    double q_inf;           // Dynamic pressure: q_inf = 0.5 * rho * V^2
    double alpha;           // Angle of Attack [rad]
    double beta;            // Angle of Sideslip [rad]
    double velocity;        // Total velocity
} AeroData;

void computeAerodynamicForces(const StateVector* X, const Actuators* actuators, const AircraftModel* acModel, AeroData* aeroData);

double computeCL(const AircraftModel* acModel, double de, double alpha);

double computeCL_wingbody(const AircraftModel* acParamms, double alpha);

double computeCL_tail(const AircraftModel* acModel, double de, double alpha);

double computeCd(double alpha);

double computeCy(double dr, double beta);

Vector3 computeCM(const AircraftModel* acModel, double alpha, double beta, double velocity, Vector3* w_b, Vector3* u_123);

double computeEpsilonDownwash(const AircraftModel* acModel, double alpha);
