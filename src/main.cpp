/*
 * main.hpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

#include "EKFSlam.hpp"
#include "Models.hpp"
#include "Log.hpp"

using namespace slam;

#define DATA_FILE "../data/sensor_data.dat"
#define WORLD_FILE "../data/world.dat"

static int loadScenario(std::vector<Data> &data,
    std::vector<Position> &landmarks)
{
    try
    {
        loadData(DATA_FILE, data);
        loadWorld(WORLD_FILE, landmarks);
        logger().info("Loaded scenario {} data elements, {} landmarks.", data.size(), landmarks.size());
    }
    catch (std::exception &e)
    {
        logger().error("Failed to load scenario: {}", e.what());
        return 1;
    }

    return 0;
}

static int run(const std::vector<Data> &data,
    const std::vector<Position> &landmarks)
{
    double sensorNoise = 0.1;
    double motionNoise = 0.1;
    Eigen::Matrix3d odomNoise;
    odomNoise << motionNoise, 0, 0,
                 0, motionNoise, 0,
                 0, 0, motionNoise/10;

    size_t dim = 3 + 2 * landmarks.size();
    EKFSlam slamAlgo(dim, odomNoise, sensorNoise);

    try
    {
        logger().info("Solving SLAM ...");
        std::vector<State> records = slamAlgo.run(data);
        logger().info("Plotting ...");
        slamAlgo.plot(records, data, landmarks, "../plot/");
    }
    catch(std::exception &e)
    {
        logger().error("Failed to run EKF: {}", e.what());
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

    return run(data, landmarks);
}
