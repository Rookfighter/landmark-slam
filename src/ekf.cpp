/*
 * ekf.hpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

#include <iostream>
#include <spdlog/spdlog.h>
#include "Models.hpp"
#include "State.hpp"
#include "Math.hpp"
#include "Log.hpp"

using namespace slam;

#define DATA_FILE "../data/sensor_data.dat"
#define WORLD_FILE "../data/world.dat"
#define INF 1000

static void predictionStep(State &state, const Odometry &odom, const OdomNoise &noise)
{
    StatePose result;
    StatePose spose = state.getPose();

    // calc motion model
    MotionModel motion = calcMotionModel(spose.pose, odom);

    // apply new pose from motion model
    result.pose = motion.val;
    // calc new covariance
    result.cov = motion.jac * spose.cov * motion.jac.transpose() + noise;

    state.setPose(result);
}

static void correctionStep(State &state, const std::vector<Observation> &observ, const Eigen::MatrixXd &noise)
{
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
            state.seen[c] = true;
        }

        // calculate sensor model
        SensorModel sensorModel = calcSensorModel(spose.pose, lm.pos);

        // transform jacobian into correct dimension
        for(size_t j = 0; j < 2; ++j)
        {
            size_t k = i*2+j;
            jacs(k,0)     = sensorModel.jac(j,0);
            jacs(k,1)     = sensorModel.jac(j,1);
            jacs(k,2)     = sensorModel.jac(j,2);
            jacs(k,idx)   = sensorModel.jac(j,3);
            jacs(k,idx+1) = sensorModel.jac(j,4);
        }

        // calculate diff of measurements
        Measurement zdiffi = obs.m - sensorModel.val;
        zdiffi(1) = normalizeAngle(zdiffi(1));

        zdiffs(i*2)   = zdiffi(0);
        zdiffs(i*2+1) = zdiffi(1);
    }

    Eigen::MatrixXd jacsT = jacs.transpose();
    Eigen::MatrixXd cov12 = state.cov * jacsT;
    Eigen::MatrixXd kalTmp = jacs * cov12 + noise;
    Eigen::MatrixXd kalGain = cov12 * kalTmp.inverse();

    state.mean = state.mean + kalGain * zdiffs;
    state.cov  = (Eigen::MatrixXd::Identity(state.dim,state.dim) - kalGain * jacs) * state.cov;
}

static int loadScenario(std::vector<Data> &data,
    std::vector<Position> &landmarks)
{
    try
    {
        loadData(DATA_FILE, data);
        loadWorld(WORLD_FILE, landmarks);
        logger()->info("Loaded scenario {} data elements, {} landmarks.", data.size(), landmarks.size());
    }
    catch (std::exception &e)
    {
        logger()->error("Failed to load scenario: {}", e.what());
        return 1;
    }

    return 0;
}

static void tryEKF(const std::vector<Data> &data,
    const std::vector<Position> &landmarks)
{
    // define noise of odometry
    double motionNoise = 0.1;
    OdomNoise odomNoise;
    odomNoise << motionNoise, 0, 0,
                 0, motionNoise, 0,
                 0, 0, motionNoise/10;

    // define noise of sensors
    double sNoise = 0.1;

    size_t dim = landmarks.size() * 2 + 3;
    // init mean estimate to be all zeros
    State state(dim);
    for(size_t i = 3; i < dim; ++i)
        state.cov(i, i) = INF;

    for(size_t i = 0; i < data.size(); ++i)
    {
        logger()->info("iteration {}", i);

        const Data &dat = data[i];

        // process prediction step of EKF
        logger()->info("  prediction step");
        predictionStep(state, dat.odom, odomNoise);

        size_t obsCount = dat.observ.size();
        // check if there were any observations
        if(obsCount > 0)
        {
            // create noise matrix for measurements
            logger()->info("  correction step");
            Eigen::MatrixXd noise = sNoise * Eigen::MatrixXd::Identity(obsCount*2, obsCount*2);
            // process correction step of EKF
            correctionStep(state, dat.observ, noise);
        }
    }
}

static int runEKF(const std::vector<Data> &data,
    const std::vector<Position> &landmarks)
{
    try
    {
        tryEKF(data, landmarks);
    }
    catch(std::exception &e)
    {
        logger()->error("Failed to run EKF: {}", e.what());
        return 1;
    }

    return 0;
}

int main()
{
    std::vector<Data> data;
    std::vector<Position> landmarks;

    int ret = loadScenario(data, landmarks);
    if(ret != 0)
        return ret;

    return runEKF(data, landmarks);
}
