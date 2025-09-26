#include "src/actuators/actuators.h"
#include "src/core/constants.h"

#include "src/io/logger.h"

/**
 * @brief Commands vehicle actuators and logs their state.
 *
 * This function takes a control vector, a pointer to the actuators, and a
 * time step to update the position of the servos and the value of the throttles.
 * It also logs the new actuator positions.
 *
 * @param[in]  U_cmd      A pointer to a ControlVector struct containing the desired control inputs.
 * @param[out] actuators  A pointer to an Actuators struct. This struct's members are
 *                        modified by the function to reflect the new state of the actuators.
 * @param[in]  dt         The time step in seconds since the last update.
 */
void driveActuators(ControlVector* U_cmd, Actuators* actuators, double dt){
    actuateServo(&(actuators->aileronServo), U_cmd->da, dt);
    actuateServo(&(actuators->elevatorServo), U_cmd->de, dt);
    actuateServo(&(actuators->rudderServo), U_cmd->dr, dt);
    actuators->throttle[0] = U_cmd->dthr[0];
    actuators->throttle[1] = U_cmd->dthr[1];

    logger.data[LOG_SRVO_AILERON_POS]   = actuators->aileronServo.pos;
    logger.data[LOG_SRVO_ELEVATOR_POS]  = actuators->elevatorServo.pos;
    logger.data[LOG_SRVO_RUDDER_POS]    = actuators->rudderServo.pos;
    logger.data[LOG_SRVO_THROTTLE1_VAL] = actuators->throttle[0];
    logger.data[LOG_SRVO_THROTTLE2_VAL] = actuators->throttle[1];
}


// double throttleLim[2]   = {0.5 * deg2rad, 10 * deg2rad};
// double dThrottleRateLim = 1.6 * deg2rad;  // rate limit for change in throttle [units/sec]
