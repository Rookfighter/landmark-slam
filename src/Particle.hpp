/*
 * Particle.hpp
 *
 *  Created on: 05 Mar 2018
 *      Author: Fabian Meyer
 */

#include "Data.hpp"

#ifndef SLAM_PARTICLE_HPP_
#define SLAM_PARTICLE_HPP_

namespace slam
{
    struct LandmarkEstimate
    {
        Position pos;
        Eigen::Matrix2d cov;
        size_t id;
    };

    struct Particle
    {
        Pose pose;
        double weight;
        std::vector<LandmarkEstimate> map;
        std::vector<bool> seen;
    };

    typedef std::vector<Particle> ParticleSet;
}

#endif
