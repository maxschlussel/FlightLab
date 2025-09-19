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


bool isEqual(double val1, double val2, double eps){
    return fabs(val1 - val2) < eps;
}


double randNoise(double stddev) {
    return stddev * ((double)rand() / RAND_MAX - 0.5);
}
