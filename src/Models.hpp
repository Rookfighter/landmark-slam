/*
 * Models.hpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

 #ifndef SLAM_MODELS_HPP_
 #define SLAM_MODELS_HPP_

#include "Data.hpp"

namespace slam
{
    struct MotionModel
    {
        typedef Pose Value;
        typedef Eigen::Matrix3d Jacobian;

        Value val;
        Jacobian jac;
    };

    MotionModel calcMotionModel(const Pose &pose, const Odometry &odom);

    struct SensorModel
    {
        typedef Measurement Value;
        typedef Eigen::Matrix<double, 2, 5> Jacobian;

        Value val;
        Jacobian jac;
    };

    SensorModel calcSensorModel(const Pose &pose, const Position &landmark);
}

#endif
