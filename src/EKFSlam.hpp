/*
 * EKFSlam.hpp
 *
 *  Created on: 05 Mar 2018
 *      Author: Fabian Meyer
 */

#ifndef SLAM_EKFSLAM_HPP_
#define SLAM_EKFSLAM_HPP_

#include "Data.hpp"
#include "State.hpp"

namespace slam
{
    class EKFSlam
    {
    private:
        size_t dim_;
        Eigen::Matrix3d odomNoise_;
        double sensorNoise_;

        void predictionStep(State &state,
            const Odometry &odom);
        void correctionStep(State &state,
            const std::vector<Observation> &observ);
    public:
        EKFSlam(const size_t dim,
                const Eigen::Matrix3d &odomNoise,
                const double sensorNoise);
        ~EKFSlam();

        std::vector<State> run(const std::vector<Data> &data);
        void plot(const std::vector<State> &records,
                  const std::vector<Data> &data,
                  const std::vector<Position> &landmarks,
                  const std::string &dir);

    };
}

#endif
