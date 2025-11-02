#include <string.h>

#include "src/aircrafts/boeing_737.h"
#include "src/core/constants.h"
#include "src/math/matrix.h"


/**
 * @brief Load and initialize the aircraft parameters for Boeing 737 model.
 *
 * This function sets up and returns an initialized instance of 
 * the AircraftModel structure, including geometry, mass 
 * properties, and aerodynamic reference values. 
 * 
 * @return AircraftModel Initialized aircraft parameter struct.
 */
AircraftModel loadBoeing737AircraftParams(void){
    AircraftModel acModel = {
        .name = "Boeing 737-800",

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
    mat3_scale(I, acModel.mass, I_scaled);

    memcpy(acModel.I, I_scaled, sizeof(I_scaled));

    // Set cg
    acModel.cg.x = 0.23 * acModel.chord;
    acModel.cg.y = 0;
    acModel.cg.z = 0.1 * acModel.chord;

    // Set ac
    acModel.ac.x = 0.12 * acModel.chord;
    acModel.ac.y = 0.0;
    acModel.ac.z = 0.0;
    
    // Calculate r_ac2cg
    acModel.r_ac2cg.x = acModel.cg.x - acModel.ac.x;
    acModel.r_ac2cg.y = acModel.cg.y - acModel.ac.y;
    acModel.r_ac2cg.z = acModel.cg.z - acModel.ac.z;
    
    // Calculate r_engineOne2cg
    acModel.r_engineOne2cg.x = acModel.cg.x - acModel.engineOneThrustPoint.x;
    acModel.r_engineOne2cg.y = -(acModel.cg.y - acModel.engineOneThrustPoint.y);
    acModel.r_engineOne2cg.z = acModel.cg.z - acModel.engineOneThrustPoint.z;
    
    // Calculate r_engineTwo2cg
    acModel.r_engineTwo2cg.x = acModel.cg.x - acModel.engineTwoThrustPoint.x;
    acModel.r_engineTwo2cg.y = -(acModel.cg.y - acModel.engineTwoThrustPoint.y);
    acModel.r_engineTwo2cg.z = acModel.cg.z - acModel.engineTwoThrustPoint.z;

    return acModel;
}