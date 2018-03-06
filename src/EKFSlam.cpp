/*
 * EKFSlam.cpp
 *
 *  Created on: 05 Mar 2018
 *      Author: Fabian Meyer
 */

#include "EKFSlam.hpp"
#include "Models.hpp"
#include "Math.hpp"
#include "Plot.hpp"

#define INF 1000

namespace slam
{
    EKFSlam::EKFSlam(const size_t dim, const Eigen::Matrix3d &odomNoise,
        const double sensorNoise)
        : dim_(dim), odomNoise_(odomNoise), sensorNoise_(sensorNoise)
    {

    }

    EKFSlam::~EKFSlam()
    {

    }

    void EKFSlam::plot(const std::vector<State> &records,
        const std::vector<Data> &data,
        const std::vector<Position> &landmarks,
        const std::string &dir)
    {
        std::stringstream ss;
        ss << dir << "ekf_";
        plotStateRecords(records, data, landmarks, ss.str());
    }

    std::vector<State> EKFSlam::run(const std::vector<Data> &data)
    {
        std::vector<State> records;
        records.reserve(data.size() + 1);

        // init mean estimate to be all zeros
        State state(dim_);
        for(size_t i = 3; i < dim_; ++i)
            state.cov(i, i) = INF;

        records.push_back(state);

        for(size_t i = 0; i < data.size(); ++i)
        {
            const Data &dat = data[i];

            // process prediction step of EKF
            predictionStep(state, dat.odom);

            size_t obsCount = dat.observ.size();
            // check if there were any observations
            if(obsCount > 0)
            {
                // create noise matrix for measurements
                // process correction step of EKF
                correctionStep(state, dat.observ);
            }

            records.push_back(state);
        }

        return records;
    }

    void EKFSlam::predictionStep(State &state,
        const Odometry &odom)
    {
        StatePose result;
        StatePose spose = state.getPose();

        // calc motion model
        MotionModel motion = calcMotionModel(spose.pose, odom);

        // apply new pose from motion model
        result.pose = motion.val;
        // calc new covariance
        result.cov = motion.jac * spose.cov * motion.jac.transpose() + odomNoise_;;

        state.setPose(result);
    }

    void EKFSlam::correctionStep(State &state,
        const std::vector<Observation> &observ)
    {
        Eigen::MatrixXd noise = sensorNoise_ * Eigen::MatrixXd::Identity(
            observ.size() * 2, observ.size() * 2);

        size_t obsCount = observ.size();

        Eigen::MatrixXd jacs   = Eigen::MatrixXd::Zero(2*obsCount, state.dim);
        Eigen::VectorXd zdiffs = Eigen::VectorXd::Zero(2*obsCount);

        for(size_t i = 0; i < observ.size(); ++i)
        {
            const Observation& obs = observ[i];
            size_t c = obs.c;
            size_t idx = 3 + 2*c;

            StateLandmark lm = state.getLandmark(c);
            StatePose spose = state.getPose();

            // check if this landmark has been seen before
            if(!state.seen[c])
            {
                InvSensorModel invSM = calcInvSensorModel(spose.pose, obs.m);
                lm.pos = invSM.val;
                state.setLandmark(lm);
                state.seen[c] = true;
            }

            // calculate sensor model
            SensorModel sensorModel = calcSensorModel(spose.pose, lm.pos);

            // calculate diff of measurements
            Measurement zdiffi = obs.m - sensorModel.val;
            zdiffi(1) = normalizeAngle(zdiffi(1));

            for(size_t j = 0; j < 2; ++j)
            {
                size_t k = i*2+j;

                zdiffs(k) = zdiffi(j);

                // transform jacobian into correct dimension
                jacs(k,0)     = sensorModel.jac(j,0);
                jacs(k,1)     = sensorModel.jac(j,1);
                jacs(k,2)     = sensorModel.jac(j,2);
                jacs(k,idx)   = sensorModel.jac(j,3);
                jacs(k,idx+1) = sensorModel.jac(j,4);
            }
        }

        Eigen::MatrixXd jacsT = jacs.transpose();
        Eigen::MatrixXd cov12 = state.cov * jacsT;
        Eigen::MatrixXd kalTmp = jacs * cov12 + noise;
        Eigen::MatrixXd kalGain = cov12 * kalTmp.inverse();

        state.mean = state.mean + kalGain * zdiffs;
        state.cov  = (Eigen::MatrixXd::Identity(state.dim,state.dim) - kalGain * jacs) * state.cov;
    }
}
