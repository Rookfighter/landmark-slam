/*
 * Models.cpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

#include "Models.hpp"
#include "Math.hpp"

namespace slam
{
    std::pair<Pose, MotionJac> motionModel(const Pose &pose, const Odometry &odom)
    {
        double dx = odom[1] * std::cos(pose[2] + odom[0]);
        double dy = odom[1] * std::sin(pose[2] + odom[0]);
        // calc delta in pose through odometry
        Eigen::Vector3d delta(
            dx,
            dy,
            odom[0] + odom[2]
        );
        // calc new pose
        Pose nPose = pose + delta;
        nPose[2] = normalizeAngle(nPose[2]);

        // calc jacobian of motion model
        MotionJac jac;
        jac << 0, 0, -dy,
               0, 0,  dx,
               0, 0,  0;

        return std::pair<Pose, MotionJac>(nPose, jac);
    }

    std::pair<Measurement,SensorJac> sensorModel(const Pose &pose, const Position &landmark)
    {
        double dx    = landmark[0] - pose[0];
        double dy    = landmark[1] - pose[1];
        double q     = dx*dx + dy *dy;
        double qSqrt = std::sqrt(q);

        // calc expected measurement
        Measurement expZ(
            qSqrt,
            std::atan2(dy, dx) - pose[3]
        );
        expZ[1] = normalizeAngle(expZ[1]);

        // calc jacobian of sensor model
        SensorJac jac;
        jac << -qSqrt * dx, -qSqrt * dy,  0,  qSqrt * dx, qSqrt * dy,
                dy,         -dx,         -q, -dy,         dx;

        return std::pair<Measurement,SensorJac>(expZ, jac);
    }

}
