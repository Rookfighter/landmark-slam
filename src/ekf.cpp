/*
 * ekf.hpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

#include <iostream>
#include "Models.hpp"

using namespace slam;

#define DATA_FILE "../data/sensor_data.dat"
#define WORLD_FILE "../data/world.dat"
#define INF 1000

typedef Eigen::VectorXd State;
typedef Eigen::MatrixXd Covariance;

static void getPose(const State &mean, const Covariance &cov, Pose &pose, PoseCov &poseCov)
{
    pose(0) = mean(0);
    pose(1) = mean(1);
    pose(2) = mean(2);

    for(size_t i = 0; i < 3; ++i)
        for(size_t j = 0; j < 3; ++j)
            poseCov(i,j) = cov(i,j);
}

static void setPose(State &mean, Covariance &cov, const Pose &pose, const PoseCov &poseCov)
{
    mean(0) = pose(0);
    mean(1) = pose(1);
    mean(2) = pose(2);

    for(size_t i = 0; i < 3; ++i)
        for(size_t j = 0; j < 3; ++j)
            cov(i,j) = poseCov(i,j);
}

static void predictionStep(State &mean, Covariance &cov, const Odometry &odom, const OdomNoise &noise)
{
    Pose pose;
    PoseCov poseCov;
    getPose(mean, cov, pose, poseCov);

    // calc motion model
    auto modelPair = motionModel(pose, odom);

    Pose nPose = modelPair.first;
    MotionJac jac = modelPair.second;

    // calc new co
    PoseCov nPoseCov = jac * poseCov * jac.transpose() + noise;

    setPose(mean, cov, nPose, nPoseCov);
}

int main()
{
    std::vector<Data> data;
    std::vector<Position> landmarks;

    try
    {
        loadSensorData(DATA_FILE, data);
        loadWorld(WORLD_FILE, landmarks);
    }
    catch (std::exception &e)
    {
        std::cerr << "Failed to load data: " << e.what() << std::endl;
        return 1;
    }

    // define noise of odometry
    double motionNoise = 0.1;
    OdomNoise odomNoise;
    odomNoise << motionNoise, 0, 0,
                 0, motionNoise, 0,
                 0, 0, motionNoise/10;

    size_t dim = landmarks.size() * 2 + 3;
    // init mean estimate to be all zeros
    State mean = State::Zero(dim);
    // init covariance matrix to have infinite uncertainty about landmark
    // locations
    Covariance cov = Covariance::Zero(dim, dim);
    for(size_t i = 2; i < dim; ++i)
        cov(i, i) = INF;

    for(size_t i = 0; i < data.size(); ++i)
    {
        const Data &dat = data[i];
        predictionStep(mean, cov, dat.odom, odomNoise);

    }

    return 0;
}
