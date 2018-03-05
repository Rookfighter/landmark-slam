/*
 * FastSlam.hpp
 *
 *  Created on: 05 Mar 2018
 *      Author: Fabian Meyer
 */

#include "Particle.hpp"

namespace slam
{
    class FastSlam
    {
    private:
        size_t dim_;
        size_t count_;
        Eigen::Matrix2d borders_;
        Eigen::Matrix2d sensorNoise_;

        void initParticles(ParticleSet &particles);
        void sampleParticles(ParticleSet &particles,
                             const Data& data);
        ParticleSet resample(const ParticleSet &particles);
    public:
        FastSlam(const size_t dim,
                 const size_t particleCount,
                 const Eigen::Matrix2d &borders,
                 const double sensorNoise);
        ~FastSlam();

        std::vector<ParticleSet> run(const std::vector<Data> &data);
        void plot(const std::vector<ParticleSet> &records,
                  const std::vector<Data> &data,
                  const std::vector<Position> &landmarks,
                  const std::string &dir);
    };
}
