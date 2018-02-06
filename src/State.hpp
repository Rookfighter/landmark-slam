/*
 * State.hpp
 *
 *  Created on: 06 Feb 2018
 *      Author: Fabian Meyer
 */

 #ifndef SLAM_STATE_HPP_
 #define SLAM_STATE_HPP_

#include "Data.hpp"

namespace slam
{
    typedef Eigen::VectorXd StateMean;
    typedef Eigen::MatrixXd StateCov;

    struct StatePose
    {
        Pose pose;
        PoseCov cov;
    };

    struct StateLandmark
    {
        Position pos;
        PositionCov cov;
        size_t c;
    };

    class State
    {
    public:
        size_t dim;
        StateMean mean;
        StateCov cov;
        std::vector<bool> seen;

        State(const size_t dim);

        StatePose getPose() const;
        void setPose(const StatePose &p);

        StateLandmark getLandmark(const size_t c) const;
        void setLandmark(const StateLandmark &lm);
    };


}

#endif
