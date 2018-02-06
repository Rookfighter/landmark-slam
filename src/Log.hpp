/*
 * Log.hpp
 *
 *  Created on: 06 Feb 2018
 *      Author: Fabian Meyer
 */

#ifndef SLAM_LOG_HPP_
#define SLAM_LOG_HPP_

#include <spdlog/spdlog.h>

namespace slam
{
    std::shared_ptr<spdlog::logger> logger();
}

#endif
