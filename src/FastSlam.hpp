/*
 * FastSlam.hpp
 *
 *  Created on: 05 Mar 2018
 *      Author: Fabian Meyer
 */

#ifndef SLAM_FASTSLAM_HPP_
#define SLAM_FASTSLAM_HPP_

#include "Particle.hpp"

namespace slam
{
    class FastSlam
    {
    private:
        size_t dim_;
        size_t count_;
        Eigen::Matrix2d borders_;
        Eigen::Matrix3d odomNoise_;
        Eigen::Matrix2d sensorNoise_;
        double defWeight_;
        std::default_random_engine rndgen_;

        void initParticles(ParticleSet &particles);
        void predictionStep(ParticleSet &particles,
                            const Data& data);
        Odometry randOdom(const Odometry &odom);
        void correctionStep(ParticleSet &particles,
                            const Data& data);
        void normalizeWeights(ParticleSet &particles);
        ParticleSet resample(const ParticleSet &particles);
    public:
        FastSlam(const size_t dim,
                 const size_t particleCount,
                 const Eigen::Matrix2d &borders,
                 const Eigen::Matrix3d &odomNoise,
                 const Eigen::Matrix2d &sensorNoise);
        ~FastSlam();

        std::vector<ParticleSet> run(const std::vector<Data> &data);
        void plot(const std::vector<ParticleSet> &records,
                  const std::vector<Data> &data,
                  const std::vector<Position> &landmarks,
                  const std::string &dir);
    };
}

#endif
