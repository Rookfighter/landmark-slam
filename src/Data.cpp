/*
 * Data.hpp
 *
 *  Created on: 05 Feb 2018
 *      Author: Fabian Meyer
 */

#include "Data.hpp"

#include <CSV.hpp>
#include <cassert>

namespace slam
{
    void loadData(const std::string &filename, std::vector<Data> &data)
    {
        csv::CsvFile csvf;
        csvf.setSep(' ');
        csvf.load(filename);

        size_t dataCount = 0;

        // first loop to check validity of file and count odoms and
        // measurements
        for(size_t i = 0; i < csvf.size(); ++i)
        {
            csv::CsvRow &row = csvf[i];

            if(row[0].asString() == "ODOMETRY")
            {
                // check if row has enough columns
                if (row.size() != 4)
                {
                    std::stringstream ss;
                    ss << "l" << i << ": invalid column count for ODOMETRY " << row.size();
                    throw std::logic_error(ss.str());
                }
                // increment odometry count
                ++dataCount;
            }
            else if(row[0].asString() == "SENSOR")
            {
                // check if row has enough columns
                if (row.size() != 4)
                {
                    std::stringstream ss;
                    ss << "l" << i << ": invalid column count for SENSOR " << row.size();
                    throw std::logic_error(ss.str());
                }
            }
            else
            {
                std::stringstream ss;
                ss << "l" << i << ": row type " << row[0].asString();
                throw std::logic_error(ss.str());
            }
        }

        // initialize odom and measurements vectors
        data.clear();
        data.reserve(dataCount);

        for(csv::CsvRow &row : csvf)
        {
            if(row[0].asString() == "ODOMETRY")
            {
                Odometry u (row[1].asDouble(),
                            row[2].asDouble(),
                            row[3].asDouble());
                Data d {u, {}};
                data.push_back(d);
            }
            else
            {
                Observation z {{row[2].asDouble(),
                               row[3].asDouble()},
                               row[1].asUInt()};

                data.back().observ.push_back(z);
            }
        }
    }

    void loadWorld(const std::string &filename, std::vector<Position> &landmarks)
    {
        csv::CsvFile csvf;
        csvf.setSep(' ');
        csvf.load(filename);

        landmarks.clear();
        landmarks.reserve(csvf.size());

        for(size_t i = 0; i < csvf.size(); ++i)
        {
            csv::CsvRow &row = csvf[i];
            if(row.size() != 3)
            {
                std::stringstream ss;
                ss << "l" << i << ": invalid column count for world " << row.size();
                throw std::logic_error(ss.str());
            }

            Position lm(row[1].asDouble(),
                        row[2].asDouble());
            landmarks.push_back(lm);
        }
    }
}
