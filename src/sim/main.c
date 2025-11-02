/**
* Main script for the flight simulation and core loop.
*/

#include <math.h>
#include <stdio.h>
#include <time.h>

#include "src/aircrafts/boeing_737.h"
#include "src/actuators/actuators.h"
#include "src/controllers/PID/PID_controller.h"
#include "src/core/aircraft_params.h"
#include "src/core/constants.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/dynamics/forces/aerodynamics.h"
#include "src/dynamics/eom.h"
#include "src/dynamics/forces/forces.h"
#include "src/dynamics/forces/gravity.h"
#include "src/dynamics/integrators/euler_integrator.h"
#include "src/dynamics/integrators/rk4.h"
#include "src/dynamics/forces/propulsion.h"
#include "src/estimators/simple_estimator.h"
#include "src/estimators/ekf.h"
#include "src/guidance/guidance.h"
#include "src/io/logger.h"
#include "src/sensors/aircraft_sensors.h"
#include "src/sim/scenarios.h"
#include "src/math/dcm.h"
#include "src/math/vector.h"


int main(int argc, char* argv[]){   
    // ---- Logger ----
    loggerInit(argv[2]);

    // ---- Global sim variables ----
    double simTime_s = 0.0;   // [s]
    double dt_s = 0.05;       // delta timestep [s]
    double tFinal_s = 200.0;  // [s]

    // ---- Initialize models ----
    AircraftModel acModel = loadBoeing737AircraftParams();
    
    StateVector X = initStateVectorBasicCruise();  // Load I.C.
    StateVector X_est = X;
    
    ControlSystemPID controlSystemPID = initControlSystemPID();
    
    Actuators actuators = initActuators(&controlSystemPID.U_cmd);
    
    GuidanceRefs guidanceRefs = initGuidanceNone();
    
    Sensors sensors = initSensors();
    SensorInput sensorInput = {&X, &actuators, &acModel, dt_s};
    
    AeroData aeroData = {{0.0}};

    EKF ekf = initEKF(&acModel, &actuators);
    
    double Xdot[12] = {0.0};

    // ---- Timer ----
    clock_t start_time = clock();

    // ---- Main loop ----
    while(simTime_s <= tFinal_s + EPS){
        loggerClear();

        // [1] Read sensors
        readSensors(&sensorInput, &sensors);

        // [2] State estimation
        estimateStateSimple(&sensors, &X_est, dt_s);

        // [3] Guidance references
        updateGuidanceRefs(&guidanceRefs);

        // [4] Compute and actuate flight controls
        PID_computeFlightControl(&X, &guidanceRefs, &acModel, &actuators, dt_s, &controlSystemPID, dt_s);  // X_est
        
        driveActuators(&controlSystemPID.U_cmd, &actuators, dt_s);

        // [5] Compute Forces and Moments
        computeForcesAndMoments(&X, &actuators, &acModel, &aeroData);
        
        // [6] Compute state derivative from EOM
        computeStateDerivative(&X, &acModel, &aeroData.F_tot, &aeroData.M_tot, Xdot);

        // [7] Log step
        loggerLogStep(simTime_s);

        // [8] Integrate one step   
        integrateRK4Step(&X, &actuators, &acModel, Xdot, dt_s);  // actuators
        // integrateEulerStep(&X, Xdot, dt_s);

        // [9] Step time
        simTime_s += dt_s;
    }

    loggerClose();

    double runTime = (double)((clock() - start_time)) / CLOCKS_PER_SEC;

    printf("Completed sim in %.3f seconds\n", runTime);
    return 0;
}
