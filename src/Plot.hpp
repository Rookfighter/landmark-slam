/*
 * Plot.hpp
 *
 *  Created on: 06 Feb 2018
 *      Author: Fabian Meyer
 */

#ifndef SLAM_PLOT_HPP_
#define SLAM_PLOT_HPP_

#include "State.hpp"

namespace slam
{
    void plotState(const State& state,
        const Data &data,
        const std::vector<Position> &landmarks,
        const std::string &filename);
    void plotRecords(const std::vector<State>& records,
        const std::vector<Data> &data,
        const std::vector<Position> &landmarks,
        const std::string &prefix);
}

#endif
