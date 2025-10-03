#include <math.h>

#include "src/math/utils.h"


/**
 * @brief Convert an array of angles from degrees to radians.
 *
 * @param[in]  deg      Input array of angles in degrees (length N).
 * @param[out] rad      Output array of angles in radians (can be same pointer as deg
 *                      for in-place conversion) (length N).
 * @param[in]  length   Number of elements in the array (N).
 */
void deg2rad_array(const double* deg, double* rad, double length){
    for (int i = 0; i < length; i++){
        rad[i] = deg[i] * deg2rad;
    }
}


/**
 * @brief Convert an array of angles from radians to degrees.
 *
 * @param[in]  rad      Input array of angles in radians (length N).
 * @param[out] deg      Output array of angles in degrees (can be same as pointer as rad
 *                      for in-place conversion) (length N).
 * @param[in]  length   Number of elements in the array (N).
 */
void rad2deg_array(const double* rad, double* deg, double length){
    for (int i = 0; i < length; i++){
        deg[i] = rad[i] * rad2deg;
    }
}


/**
 * @brief Clamp a value to a given range.
 *
 * Returns @p min if @p x is less than @p min ,  
 * @p max if @p x is greater than @p max , 
 * otherwise returns @p x unchanged.
 *
 * @param[in]  x    Input value
 * @param[in]  min  Lower bound
 * @param[in]  max  Upper bound
 * @return     Clamped value
 */
double clamp(double x, double min, double max){
    if (x < min) return min;
    else if (x > max) return max;
    else return x;
}


/**
 * @brief Checks if two double-precision floating-point numbers are approximately equal.
 *
 * @param[in]  val1   First double value
 * @param[in]  val2   Second double value
 * @param[in]  eps    The maximum absolute difference allowed for the values to be considered equal. 
 * @return true bool if the absolute difference between val1 and val2 is less than eps, otherwise false.
 */
bool isEqual(double val1, double val2, double eps){
    return fabs(val1 - val2) < eps;
}


/**
 * @brief Generates a pseudo-random number with a uniform distribution centered at zero.
 *
 * This function calculates a random value in the range [-stddev/2, +stddev/2].
 * It is NOT a Gaussian (Normal) distribution, but a simple uniform random value 
 * scaled by the standard deviation parameter.
 *
 * @param stddev The scaling factor that determines the width of the random range.
 *               The generated noise will be between -(stddev/2) and +(stddev/2).
 * @return A double-precision pseudo-random value.
 */
double randNoise(double stddev) {
    return stddev * ((double)rand() / RAND_MAX - 0.5);
}


/**
 * @brief Computes the weighted average of two numbers.
 * Formula: output = alpha * valA + (1.0 - alpha) * valB
 *
 * @param valA      The first value.
 * @param valB      The second value.
 * @param alpha     The weighting factor, between [0.0, 1.0].
 * @return          The weighted average value.
 */
double weightedAverage(double valA, double valB, double alpha){
    return alpha * valA + (1.0 - alpha) * valB;
}


/**
 * @brief Implements a first-order exponential weighted average low-pass filter.
 *
 * @param signal     The current, raw input signal value.
 * @param prevSignal The previous output of the filter (the previously smoothed value).
 * @param alpha      The smoothing factor, between [0.0, 1.0].
 * @return           The new filtered signal value.
 */
double lowPassFilter(double signal, double prevSignal, double alpha){
    return weightedAverage(prevSignal, signal, alpha);
}


/**
 * @brief Normalizes an angle (in radians) to a specified 2*PI periodic range (lower, upper).
 *
 * @param angle The input angle value in radians.
 * @param lower The lower bound of the target range.
 * @param upper The upper bound of the target range.
 * @return The normalized angle in radians, guaranteed to be in the range [lower, upper),
 *
 * @note Must satisfy: upper - lower is equal to 2*PI.
 */
double wrapAngle(double angle, double lower, double upper) {
    double range_width = upper - lower;

    if (fabs(range_width - twoPi) > EPS) {
        // If the range is invalid, return the original angle
        return angle;
    }

    while (angle > upper) angle -= twoPi;
    while (angle < lower) angle += twoPi;
    return angle;
}
