/*
 * Math.cpp
 *
 *  Created on: 18 Jan 2018
 *      Author: Fabian Meyer
 */

#include "Math.hpp"

namespace slam
{
    bool equals(const float a, const float b, const float eps)
    {
        return (std::fabs(a) - std::fabs(b)) < eps;
    }

    bool equals(const double a, const double b, const double eps)
    {
        return (std::fabs(a) - std::fabs(b)) < eps;
    }

    double normalizeAngle(double angle)
    {
        while(angle <= -pi())
            angle += 2*pi();
        while(angle > pi())
            angle -= 2*pi();
        return angle;
    }

    float normalizeAngle(float angle)
    {
        while(angle <= -pif())
            angle += 2*pif();
        while(angle > pif())
            angle -= 2*pif();
        return angle;
    }
}
