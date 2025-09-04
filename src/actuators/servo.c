#include "src/actuators/servo.h"
#include "src/math/utils.h"


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
