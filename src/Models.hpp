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
    typedef Eigen::Matrix3d MotionJac;
    std::pair<Pose, MotionJac> motionModel(const Pose &pose, const Odometry &odom);

    typedef Eigen::Matrix<double, 2, 5> SensorJac;
    std::pair<Measurement,SensorJac> sensorModel(const Pose &pose, const Position &landmark);
}

#endif
