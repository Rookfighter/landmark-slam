/*
 * State.cpp
 *
 *  Created on: 06 Feb 2018
 *      Author: Fabian Meyer
 */

#include "State.hpp"

namespace slam
{
    State::State(const size_t dim)
    : dim(dim), mean(StateMean::Zero(dim)), cov(StateCov::Zero(dim, dim)), seen(dim, false)
    {

    }

    StatePose State::getPose() const
    {
        StatePose result;

        result.pose << mean(0),
                       mean(1),
                       mean(2);

        result.cov << cov(0,0), cov(0,1), cov(0,2),
                      cov(1,0), cov(1,1), cov(1,2),
                      cov(2,0), cov(2,1), cov(2,2);

        return result;
    }

    void State::setPose(const StatePose &p)
    {
        mean(0) = p.pose(0);
        mean(1) = p.pose(1);
        mean(2) = p.pose(2);

        for(size_t i = 0; i < 3; ++i)
            for(size_t j = 0; j < 3; ++j)
                cov(i,j) = p.cov(i,j);
    }

    StateLandmark State::getLandmark(const size_t c) const
    {
        assert((3 + 2*c) + 1 < static_cast<size_t>(mean.rows()));

        StateLandmark result;
        size_t idx = 3 + c*2;

        result.pos << mean(idx),
                      mean(idx+1);

        result.cov << cov(idx, idx),  cov(idx, idx+1),
                      cov(idx+1,idx), cov(idx+1,idx+1);

        result.c = c;

        return result;
    }

    void State::setLandmark(const StateLandmark &lm)
    {
        assert((3 + 2*lm.c) + 1 < static_cast<size_t>(mean.rows()));

        size_t idx = 3 + 2*lm.c;

        mean(idx)   = lm.pos(0);
        mean(idx+1) = lm.pos(1);

        cov(idx, idx)     = lm.cov(0,0);
        cov(idx, idx+1)   = lm.cov(0,1);
        cov(idx+1, idx)   = lm.cov(1,0);
        cov(idx+1, idx+1) = lm.cov(1,1);
    }
}
