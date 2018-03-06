/*
 * FastSlam.cpp
 *
 *  Created on: 05 Mar 2018
 *      Author: Fabian Meyer
 */

#include "FastSlam.hpp"
#include "Models.hpp"
#include "Math.hpp"
#include "Plot.hpp"

namespace slam
{
    FastSlam::FastSlam(const size_t dim,
        const size_t particleCount,
        const Eigen::Matrix3d &odomNoise,
        const Eigen::Matrix2d &sensorNoise)
    : dim_(dim), count_(particleCount),
      odomNoise_(odomNoise),sensorNoise_(sensorNoise),
      defWeight_(1.0 / particleCount)
    {

    }

    FastSlam::~FastSlam()
    {

    }

    void FastSlam::plot(const std::vector<ParticleSet> &records,
              const std::vector<Data> &data,
              const std::vector<Position> &landmarks,
              const std::string &dir)
    {
        std::stringstream ss;
        ss << dir << "fast_";
        plotParticleRecords(records, data, landmarks, ss.str());
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
            const Data &dat = data[i];

            // move particles according to motion model
            predictionStep(particles, dat);
            // improve map and calc weight according to sensor model
            correctionStep(particles, dat);
            // normalize weights of particles such that their sum is 1
            normalizeWeights(particles);
            // resample particles
            particles = resample(particles);

            records.push_back(particles);
        }

        return records;
    }

    void FastSlam::initParticles(ParticleSet &particles)
    {
        size_t landmarkCount = (dim_ - 3) / 2;

        for(Particle &p :particles)
        {
            p.pose(0) = 0;
            p.pose(1) = 0;
            p.pose(2) = 0;
            p.weight = defWeight_;

            p.map.resize(landmarkCount);
            p.seen.resize(landmarkCount);
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

    void FastSlam::predictionStep(ParticleSet &particles,
        const Data& data)
    {
        for(Particle &p: particles)
        {
            // get a normal distributed random odometry measurement
            Odometry odom = randOdom(data.odom);
            // use this as input for motion model
            MotionModel mm = calcMotionModel(p.pose, odom);
            // apply new pose
            p.pose = mm.val;
        }
    }

    Odometry FastSlam::randOdom(const Odometry &odom)
    {
        std::normal_distribution<double> distRot1(odom(0), odomNoise_(0,0));
        std::normal_distribution<double> distTrans(odom(1), odomNoise_(1,1));
        std::normal_distribution<double> distRot2(odom(2), odomNoise_(2,2));

        Odometry result;
        result << distRot1(rndgen_),
                  distTrans(rndgen_),
                  distRot2(rndgen_);

        return result;
    }

    void FastSlam::correctionStep(ParticleSet &particles,
        const Data& data)
    {
        for(Particle &p: particles)
        {
            for(const Observation &obs: data.observ)
            {
                size_t c = obs.c;
                LandmarkEstimate &lm = p.map[c];

                // check if the landmark has been seen before
                if(p.seen[c])
                {
                    // do EKF update for landmark
                    SensorModelLm sm = calcSensorModelLm(p.pose, lm.pos);
                    Eigen::Matrix2d jacT = sm.jac.transpose();

                    Eigen::Matrix2d sensorCov = sm.jac * lm.cov * jacT +  sensorNoise_;
                    Eigen::Matrix2d sensorCovInv = sensorCov.inverse();
                    // calc kalman gain
                    Eigen::Matrix2d kalGain = lm.cov * jacT * sensorCovInv;
                    Eigen::Vector2d diff = obs.m - sm.val;
                    Eigen::Matrix<double, 1, 2> diffT = diff.transpose();

                    // calc new landmark estimate
                    lm.pos = lm.pos + kalGain * diff;
                    lm.cov = (Eigen::Matrix2d::Identity(2,2) - kalGain * sm.jac) * lm.cov;

                    // calculate new weight as PDF
                    Eigen::Matrix2d normMat = 2 * pi() * sensorCov;
                    double normalizer = 1.0 / std::sqrt(normMat.lpNorm<1>());
                    p.weight = normalizer * std::exp(-0.5 * diffT * sensorCovInv * diff);
                }
                else
                {
                    // use inverse sensor model to initialize landmark pos
                    InvSensorModel ism = calcInvSensorModel(p.pose, obs.m);
                    lm.pos = ism.val;

                    // retrieve jacobian to calc initial covariance of landmark
                    SensorModelLm sm = calcSensorModelLm(p.pose, lm.pos);
                    Eigen::Matrix2d jacInv = sm.jac.inverse();
                    lm.cov = jacInv * sensorNoise_ * jacInv.transpose();

                    // set weight to some predefined default weight
                    p.weight = defWeight_;
                    p.seen[c] = true;
                }
            }
        }
    }

    static double sumWeights(const ParticleSet &particles)
    {
        double sum = 0;
        for(const Particle &p: particles)
            sum += p.weight;

        return sum;
    }

    void FastSlam::normalizeWeights(ParticleSet &particles)
    {
        double sum = sumWeights(particles);

        for(Particle &p: particles)
            p.weight /= sum;
    }

    ParticleSet FastSlam::resample(const ParticleSet &particles)
    {
        ParticleSet result(particles.size());

        // calc step between each sample
        // assuming normalized weights (sum is 1.0)
        double step =  1.0 / particles.size();

        // calc a random starting point for resampling
        std::uniform_real_distribution<double> distrib(0, 1.0);
        double position = distrib(rndgen_);

        // init accumulated weights with weight of first particle
        double accumWeight = particles[0].weight;
        size_t k = 0;

        // do low variance resampling (stochastic universal resampling)
        for(size_t i = 0; i < particles.size(); ++i)
        {
            // check if accumulated weight is less than current sample
            // increment particle number k until not satisfied anymore
            while(accumWeight < position)
            {
                k = (k + 1) % particles.size();
                accumWeight += particles[k].weight;
            }

            result[i] = particles[k];
            result[i].weight = defWeight_;
            position += step;
        }

        return result;
    }
}
