/*
 * Math.hpp
 *
 *  Created on: 18 Jan 2018
 *      Author: Fabian Meyer
 */

#ifndef SLAM_MATH_HPP_
#define SLAM_MATH_HPP_

#include <cmath>

namespace slam
{
    constexpr double pi() { return std::atan(1.0)*4.0; }
    constexpr float pif() { return std::atan(1.0f)*4.0f; }

    /**
     * Checks if the given values (only for real numbers) are the same.
     *
     * @return true if the difference between a and b is less than eps, else false
     */
    bool equals(const float a, const float b, const float eps);
    bool equals(const double a, const double b, const double eps);

    /**
     * Normalizes the given angle (radian) to the intervall [-pi;pi].
     *
     * @param angle angle to be normalized
     * @return noramlized angle
     */
    double normalizeAngle(double angle);
    float normalizeAngle(float angle);
}

#endif
