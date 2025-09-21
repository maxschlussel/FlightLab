#pragma once

#include "src/core/control_vector.h"
#include "src/core/state_vector.h"
#include "src/controllers/PID/PID_controller.h"


StateVector initStateVectorBasicCruise(void);

ControlSystemPID initControlSystemPID(void);

Actuators initActuators(ControlVector* U_cmd);

