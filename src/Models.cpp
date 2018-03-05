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
    MotionModel calcMotionModel(const Pose &pose, const Odometry &odom)
    {
        MotionModel result;
        double dx = odom(1) * std::cos(pose(2) + odom(0));
        double dy = odom(1) * std::sin(pose(2) + odom(0));
        // calc delta in pose through odometry
        Eigen::Vector3d delta(
            dx,
            dy,
            odom(0) + odom(2)
        );
        // calc new pose
        result.val = pose + delta;
        result.val(2) = normalizeAngle(result.val(2));

        // calc jacobian of motion model
        result.jac << 1, 0, -dy,
                      0, 1,  dx,
                      0, 0,  1;

        return result;
    }

    SensorModel calcSensorModel(const Pose &pose, const Position &landmark)
    {
        SensorModel result;
        double dx    = landmark(0) - pose(0);
        double dy    = landmark(1) - pose(1);
        double q     = dx * dx + dy * dy;
        double qSqrt = std::sqrt(q);

        // calc expected measurement
        result.val << qSqrt,
                      std::atan2(dy, dx) - pose(2);
        result.val(1) = normalizeAngle(result.val(1));

        // calc jacobian of sensor model
        result.jac << -qSqrt * dx, -qSqrt * dy,  0,  qSqrt * dx, qSqrt * dy,
                       dy,         -dx,         -q, -dy,         dx;

        result.jac = result.jac * (1/q);

        return result;
    }

    SensorModelLm calcSensorModelLm(const Pose &pose, const Position &landmark)
    {
        SensorModelLm result;
        double dx    = landmark(0) - pose(0);
        double dy    = landmark(1) - pose(1);
        double q     = dx * dx + dy * dy;
        double qSqrt = std::sqrt(q);

        // calc expected measurement
        result.val << qSqrt,
                      std::atan2(dy, dx) - pose(2);
        result.val(1) = normalizeAngle(result.val(1));

        // calc jacobian of sensor model
        result.jac << dx / result.val(0),
                      dy / result.val(0),
                      -dy / (result.val(0) * result.val(0)),
                      dx / (result.val(0) * result.val(0));

        return result;
    }

    InvSensorModel calcInvSensorModel(const Pose &pose, const Measurement &measurement)
    {
        InvSensorModel result;

        double dx = measurement(0) * std::cos(pose(2) + measurement(1));
        double dy = measurement(0) * std::sin(pose(2) + measurement(1));

        result.val <<
            pose(0) + dx,
            pose(1) + dy;

        result.jac = InvSensorModel::Jacobian::Zero();

        return result;
    }

}
