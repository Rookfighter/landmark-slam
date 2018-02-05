/*
 * ekf.hpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

#include <iostream>
#include "Data.hpp"

using namespace slam;

#define DATA_FILE "../data/sensor_data.dat"
#define WORLD_FILE "../data/world.dat"

int main()
{
    std::vector<Position> landmarks;
    std::vector<Observation> observ;
    std::vector<Odometry> odom;

    try
    {
        loadSensorData(DATA_FILE, odom, observ);
        loadWorld(WORLD_FILE, landmarks);
    }
    catch (std::exception &e)
    {
        std::cerr << "Failed to load data: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
