#include <math.h>

#include "src/math/dcm.h"
/**
 * @brief Computes a 3x3 rotation matrix from given Euler angles (yaw, pitch, roll) [rad].
 *
 * This function generates a 3x3 direction cosine matrix (DCM) corresponding
 * to a sequence of intrinsic rotations about the body axes. The order of
 * rotation is Z-Y-X:
 *   - Yaw   (psi)   : rotation about Z-axis
 *   - Pitch (theta) : rotation about Y-axis
 *   - Roll  (phi)   : rotation about X-axis
 *
 * @param[in]  psi    Yaw angle in radians (rotation about Z-axis).
 * @param[in]  theta  Pitch angle in radians (rotation about Y-axis).
 * @param[in]  phi    Roll angle in radians (rotation about X-axis).
 * @param[out] R      3x3 rotation matrix.
 *
 * @note All angles must be provided in radians.
 *
 * @code
 *   double R[3][3];
 *   getRotationMatrix(psi, theta, phi, R);
 * @endcode
 */

void getRotationMatrix(double phi, double theta, double psi, double R[3][3]){
    // Calculate trigenometric constants
    double cosPhi   = cos(phi), sinPhi = sin(phi);
    double cosTheta = cos(theta), sinTheta = sin(theta);
    double cosPsi   = cos(psi), sinPsi = sin(psi);
    
    // Populate rotation matrix
    R[0][0] = cosTheta * cosPsi;
    R[0][1] = cosTheta * sinPsi;
    R[0][2] = -sinTheta;
    
    R[1][0] = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
    R[1][1] = sinPhi * sinTheta * sinPsi + cosPhi * cosPsi;
    R[1][2] = sinPhi * cosTheta;
    
    R[2][0] = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
    R[2][1] = cosPhi * sinTheta * sinPsi - sinPhi * cosPsi;
    R[2][2] = cosPhi * cosTheta;
}
