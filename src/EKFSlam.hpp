/*
 * EKFSlam.hpp
 *
 *  Created on: 05 Mar 2018
 *      Author: Fabian Meyer
 */


#include "Data.hpp"
#include "State.hpp"

namespace slam
{
    class EKFSlam
    {
    private:
        Eigen::Matrix3d odomNoise_;
        double sensorNoise_;

        void predictionStep(State &state,
            const Odometry &odom);
        void correctionStep(State &state,
            const std::vector<Observation> &observ);
    public:
        EKFSlam(const Eigen::Matrix3d &odomNoise,
            const double sensorNoise);
        ~EKFSlam();

        std::vector<State> run(const std::vector<Data> &data,
                               const std::vector<Position> &landmarks);
        void plot(const std::vector<State> &records,
                  const std::vector<Data> &data,
                  const std::vector<Position> &landmarks,
                  const std::string &dir);

    };
}
