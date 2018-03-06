/*
 * main.hpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

#include <chrono>
#include "EKFSlam.hpp"
#include "FastSlam.hpp"
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

// static int run(const std::vector<Data> &data,
//     const std::vector<Position> &landmarks)
// {
//     double sensorNoise = 0.1;
//     double motionNoise = 0.1;
//     Eigen::Matrix3d odomNoise;
//     odomNoise << motionNoise, 0, 0,
//                  0, motionNoise, 0,
//                  0, 0, motionNoise/10;
//
//     size_t dim = 3 + 2 * landmarks.size();
//     EKFSlam slamAlgo(dim, odomNoise, sensorNoise);
//
//     try
//     {
//         logger().info("Solving SLAM ...");
//         std::vector<State> records = slamAlgo.run(data);
//         logger().info("Plotting ...");
//         slamAlgo.plot(records, data, landmarks, "../plot/");
//     }
//     catch(std::exception &e)
//     {
//         logger().error("Failed to run EKF: {}", e.what());
//         return 1;
//     }
//
//     return 0;
// }

static int run(const std::vector<Data> &data,
    const std::vector<Position> &landmarks)
{
    Eigen::Matrix2d sensorNoise;
    sensorNoise << 0.1,   0,
                     0, 0.1;
    Eigen::Matrix3d odomNoise;
    odomNoise << 0.005,    0,     0,
                     0, 0.01,     0,
                     0,    0, 0.005;
    Eigen::Matrix2d borders;
    borders << 0, 10,
               0, 10;

    size_t dim = 3 + 2 * landmarks.size();
    FastSlam slamAlgo(dim, 100, borders, odomNoise, sensorNoise);

    try
    {
        logger().info("Solving SLAM ...");

        auto start = std::chrono::system_clock::now();
        std::vector<ParticleSet> records = slamAlgo.run(data);
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
        logger().info("Done in {}ms!", duration.count());

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
