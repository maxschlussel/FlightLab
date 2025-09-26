#include <string.h>

#include "src/aircrafts/boeing_737.h"
#include "src/core/constants.h"
#include "src/math/matrix.h"


/**
 * @brief Load and initialize the aircraft parameters for Boeing 737 model.
 *
 * This function sets up and returns an initialized instance of 
 * the AircraftParams structure, including geometry, mass 
 * properties, and aerodynamic reference values. 
 * 
 * @return AircraftParams Initialized aircraft parameter struct.
 */
AircraftParams loadBoeing737AircraftParams(void){
    AircraftParams acParams = {
        .mass = 120000.0,

        .chord = 6.6,
        .l_t = 24.8,
        .S = 260.0,
        .S_tail = 64.0,

        .engineOneThrustPoint = {0, -7.94, -1.9},
        .engineTwoThrustPoint = {0,  7.94, -1.9},

        .dEpsDa = 0.25,

        .alpha_L0 = -11.5 * deg2rad,
        .slope_CL_Alpha = 5.5,
        .a3 = -768.5,
        .a2 = 609.2,
        .a1 = -155.2,
        .a0 = 15.212,
        .alphaNonlinear = 14.5 * deg2rad
    };
    
    // Set I
    double I[3][3] = {
        {40.7, 0.0,   0.0},
        {0.0,  64.0,  0.0},
        {0.0,  0.0,   99.92}
    };
    
    double I_scaled[3][3];
    mat3_scale(I, acParams.mass, I_scaled);

    memcpy(acParams.I, I_scaled, sizeof(I_scaled));

    // Set cg
    acParams.cg.x = 0.23 * acParams.chord;
    acParams.cg.y = 0;
    acParams.cg.z = 0.1 * acParams.chord;

    // Set ac
    acParams.ac.x = 0.12 * acParams.chord;
    acParams.ac.y = 0.0;
    acParams.ac.z = 0.0;
    
    // Calculate r_ac2cg
    acParams.r_ac2cg.x = acParams.cg.x - acParams.ac.x;
    acParams.r_ac2cg.y = acParams.cg.y - acParams.ac.y;
    acParams.r_ac2cg.z = acParams.cg.z - acParams.ac.z;
    
    // Calculate r_engineOne2cg
    acParams.r_engineOne2cg.x = acParams.cg.x - acParams.engineOneThrustPoint.x;
    acParams.r_engineOne2cg.y = -(acParams.cg.y - acParams.engineOneThrustPoint.y);
    acParams.r_engineOne2cg.z = acParams.cg.z - acParams.engineOneThrustPoint.z;
    
    // Calculate r_engineTwo2cg
    acParams.r_engineTwo2cg.x = acParams.cg.x - acParams.engineTwoThrustPoint.x;
    acParams.r_engineTwo2cg.y = -(acParams.cg.y - acParams.engineTwoThrustPoint.y);
    acParams.r_engineTwo2cg.z = acParams.cg.z - acParams.engineTwoThrustPoint.z;

    return acParams;
}