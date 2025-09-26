#include "src/actuators/servo.h"
#include "src/math/utils.h"

/**
 * @brief Updates a servo's position based on a command input and the servo loop time step.
 *
 * This function actuates the servo towards the commanded new positon using a fisrt order
 * lag system. It applies a rate limit to ensure the servo does not move faster than its
 * physical capability and checks for saturation at the minimum and maximum position limits.
 *
 * @param[in,out] servo  A pointer to the ServoActuator struct to be updated.
 * @param[in]     U_cmd  The commanded position for the servo [rad].
 * @param[in]     dt     The time step since the last update in seconds.
 */
void actuateServo(ServoActuator* servo, double U_cmd, double dt){
    // Compute target position
    double err = U_cmd - servo->pos;
    double correction = err * dt / servo->tau;

    // Ensure max rates are not passed
    double maxStep = servo->rateLim * dt;
    double step = clamp(correction, -maxStep, maxStep);

    // Update position
    servo->pos += step;

    // Check saturation
    servo->pos = clamp(servo->pos, servo->minPosLim, servo->maxPosLim);
}
