/**
 * Objective:
 *     Propogate 6DOF motion of an aircraft
 * 
 * Input:
 *      1. Aircraft params
 *      2. Initial condition state
 *      3. Control inputs
 * 
 * Output:
 *     1. Propogated State timeseries
 *     2. Propogated velocity/rates timeseries
 * 
 * Steps:
 *     1. Declare sim variables and initialize sim models
 *     2. Begin main sim loop:
 *        2.1 Compute Forces and Moments
 *        2.2 Compute state derivative from EOM
 *        2.3 Integrate one step (Euler, RK4, etc.) & step time
 *        2.4 Log and display results
 *        2.5 Step time
 * 
 * Assumptions:
 *      1. Constant Mass and Inertia
 *      2. Constant alt density and gravity
 *      3. Flat Earth EOM
*/

#include <math.h>
#include <stdio.h>
#include <time.h>

#include "src/aircrafts/boeing_737.h"
#include "src/actuators/flight_controls.h"
#include "src/controllers/basic_PID.h"
#include "src/core/aircraft_params.h"
#include "src/core/constants.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/dynamics/aerodynamics.h"
#include "src/dynamics/eom.h"
#include "src/dynamics/forces.h"
#include "src/dynamics/gravity.h"
#include "src/dynamics/integrators/euler_integrator.h"
#include "src/dynamics/integrators/rk4.h"
#include "src/dynamics/propulsion.h"
#include "src/io/logger.h"
#include "src/sensors/sensors.h"
#include "src/sim/scenarios.h"
#include "src/math/dcm.h"
#include "src/math/vector.h"


int main(int argc, char* argv[]){   
    loggerInit(argv[1]);

    // Global sim variables
    double simTime_s = 0.0;   // [s]
    double dt_s = 0.05;       // timestep [s]
    double tFinal_s = 200.0;  // [s]

    Vector3 F_tot = {0.0}, M_tot = {0.0};

    // Initialize models
    AircraftParams acParams = loadBoeing737AircraftParams();

    StateVector X = initStateVectorBasicCruise();

    ControlVector U = initControlVectorlBasic();

    Sensors sensors = initSensors();

    FlightControls flightControls = initFlightControls(&U);

    double Xdot[12] = {0.0};

    // Timer
    clock_t start_time = clock();

    while(simTime_s <= tFinal_s + EPS){
        loggerClear();

        // [1] Read sensors
        readSensors(&X, &sensors);

        // [2] Compute and actuate flight controls
        computeFlightControls(&sensors, &U, &acParams);
        actuateFlightControls(&U, &flightControls, dt_s);

        // [3] Compute Forces and Moments
        computeForcesAndMoments(&X, &U, &acParams, &F_tot, &M_tot);
        
        // [4] Compute state derivative from EOM
        computeStateDerivative(&X, &acParams, &F_tot, &M_tot, Xdot);

        // [5] Log step
        loggerLogStep(simTime_s);

        // [6] Integrate one step   
        integrateRK4Step(&X, &U, &acParams, Xdot, dt_s);  // integrateEulerStep(&X, Xdot, dt_s);

        // [7] Step time
        simTime_s += dt_s;
    }

    loggerClose();

    double runTime = (double)((clock() - start_time)) / CLOCKS_PER_SEC;

    printf("Completed sim in %.3f seconds\n", runTime);
    return 0;
}
