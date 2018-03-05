/*
 * FastSlam.cpp
 *
 *  Created on: 05 Mar 2018
 *      Author: Fabian Meyer
 */

#include "FastSlam.hpp"
#include "Models.hpp"
#include "Math.hpp"
#include "Log.hpp"

namespace slam
{
    FastSlam::FastSlam(const size_t dim,
        const size_t particleCount,
        const Eigen::Matrix2d &borders,
        const double sensorNoise)
    : dim_(dim), count_(particleCount), borders_(borders), sensorNoise_(sensorNoise * Eigen::Matrix2d::Identity(2,2))
    {

    }

    FastSlam::~FastSlam()
    {

    }

    std::vector<ParticleSet> FastSlam::run(const std::vector<Data> &data)
    {
        std::vector<ParticleSet> records;
        records.reserve(data.size() + 1);

        ParticleSet particles(count_);
        initParticles(particles);

        records.push_back(particles);

        for(size_t i = 0; i < data.size(); ++i)
        {
            logger().info("iteration {}", i);

            const Data &dat = data[i];

            sampleParticles(particles, dat);

        }

        return records;
    }

    void FastSlam::initParticles(ParticleSet &particles)
    {
        size_t landmarkCount = (dim_ - 3) / 2;
        std::default_random_engine generator;
        std::uniform_real_distribution<double> distribX(borders_(0,0),borders_(0,1));
        std::uniform_real_distribution<double> distribY(borders_(1,0),borders_(1,1));

        for(Particle &p :particles)
        {
            p.pose(0) = distribX(generator);
            p.pose(1) = distribY(generator);
            p.pose(2) = 0;
            p.weight = 0;

            p.map.resize(landmarkCount);
            for(size_t j = 0; j < p.map.size(); ++j)
            {
                LandmarkEstimate &lm = p.map[j];

                lm.pos << 0, 0;
                lm.cov << 0, 0,
                          0, 0;
                lm.id = j;
                p.seen[j] = false;
            }
        }
    }

    void FastSlam::sampleParticles(ParticleSet &particles,
                         const Data& data)
    {
        double defWeight = 1 / particles.size();

        for(Particle &p: particles)
        {
            MotionModel mm = calcMotionModel(p.pose, data.odom);
            p.pose = mm.val;

            for(const Observation &obs: data.observ)
            {
                size_t c = obs.c;
                LandmarkEstimate &lm = p.map[c];
                if(p.seen[c])
                {
                    SensorModelLm sm = calcSensorModelLm(p.pose, lm.pos);

                    Eigen::Matrix2d jacT = sm.jac.transpose();
                    Eigen::Matrix2d sensorCov = sm.jac * lm.cov * jacT +  sensorNoise_;
                    Eigen::Matrix2d sensorCovInv = sensorCov.inverse();
                    Eigen::Matrix2d kalGain = lm.cov * jacT * sensorCovInv;
                    Eigen::Vector2d diff = obs.m - sm.val;
                    lm.pos = lm.pos + kalGain * diff;
                    lm.cov = (Eigen::Matrix2d::Identity(2,2) - kalGain * sm.jac) * lm.cov;
                    double normalizer = 1 / std::sqrt((2 * pi() * sensorCov).determinant());

                    p.weight = normalizer * std::exp(-0.5 * diff.transpose() * sensorCovInv * diff);
                }
                else
                {
                    InvSensorModel ism = calcInvSensorModel(p.pose, obs.m);
                    lm.pos = ism.val;
                    SensorModelLm sm = calcSensorModelLm(p.pose, lm.pos);

                    Eigen::Matrix2d jacInv = sm.jac.inverse();
                    lm.cov = jacInv * sensorNoise_ * jacInv.transpose();
                    p.weight = defWeight;
                }
            }
        }
    }
}
