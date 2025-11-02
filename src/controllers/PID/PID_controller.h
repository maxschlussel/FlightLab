# pragma once

#include "src/actuators/actuators.h"
#include "src/controllers/PID/PID.h"
#include "src/core/aircraft_params.h"
#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/guidance/guidance.h"


/**
 * @brief Represents the flight control system's commands and PID controllers.
 *
 * This struct encapsulates the entire flight control system, including final
 * computed control commands for the aircraft's actuators, as well as the all
 * the necessary PID controllers for the longitudinal and lateral flight 
 * control loops.
 */
typedef struct {
    ControlVector U_cmd;

    PID alt2pitchPID;
    PID pitch2elvPID;
    PID airspeed2throtPID;
    
    PID heading2rollPID;
    PID roll2aileronPID;
} ControlSystemPID;


void PID_computeFlightControl(const StateVector* X_est, const GuidanceRefs* guidanceRefs, const AircraftModel* acModel, 
                             Actuators* actuators, double dt, ControlSystemPID* controlSystemPID, double dt_s);
