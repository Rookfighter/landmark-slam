/*
 * Data.hpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

#ifndef SLAM_DATA_HPP_
#define SLAM_DATA_HPP_

#include <Eigen/Dense>

namespace slam
{
    // 2D pose (x, y, theta)
    typedef Eigen::Vector3d Pose;
    typedef Eigen::Matrix3d PoseCov;

    // range bearing measurement (r, theta, c)
    typedef Eigen::Vector2d Measurement;

    // observation with measurement and data assoc
    typedef struct {
        Measurement m;
        size_t c;
    } Observation;

    // odometry (r1, t, r2)
    typedef Eigen::Vector3d Odometry;

    // 2D position (x, y)
    typedef Eigen::Vector2d Position;
    typedef Eigen::Matrix2d PositionCov;

    typedef struct {
        Odometry odom;
        std::vector<Observation> observ;
    } Data;

    void loadData(const std::string &filename, std::vector<Data> &data);
    void loadWorld(const std::string &filename, std::vector<Position> &landmarks);

}

#endif
