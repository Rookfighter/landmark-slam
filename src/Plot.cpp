
#include "Plot.hpp"
#include "Math.hpp"
#include "Log.hpp"
#include <sstream>
#include <iomanip>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

namespace slam
{
    static void plotPose(const Pose &pose)
    {
        double radius = 0.3;
        size_t corners = 16;
        double step = (pi() / corners);

        std::vector<double> x(corners);
        std::vector<double> y(corners);

        for(size_t i = 0; i < corners; ++i)
        {
                x[i] = pose(0) + radius * cos(i * step);
                y[i] = pose(1) + radius * sin(i * step);
        }

        double sx = pose(0);
        double sy = pose(1);
        double tx = pose(0) + radius * cos(pose(2));
        double ty = pose(1) + radius * sin(pose(2));

        plt::plot(x, y, "r-");
        plt::plot({sx, tx}, {sy, ty}, "r-");
    }

    void plotState(const State& state, const std::vector<Position> &landmarks, const std::string &filename)
    {
        plt::figure();

        std::vector<double> lmsX(landmarks.size());
        std::vector<double> lmsY(landmarks.size());

        for(size_t i = 0; i < landmarks.size(); ++i)
        {
            lmsX[i] = landmarks[i](0);
            lmsY[i] = landmarks[i](1);
        }

        logger().info("plot lms");
        plt::plot(lmsX, lmsY, "b*");
        logger().info("plot pose");
        plotPose(state.getPose().pose);

        plt::save(filename);
    }

    void plotRecords(const std::vector<State>& records, const std::vector<Position> &landmarks, const std::string &prefix)
    {
        for(size_t i = 0; i < records.size(); ++i)
        {
            std::stringstream ss;
            ss << prefix << std::setfill('0') << std::setw(3) << i+1 << ".png";
            plotState(records[i], landmarks, ss.str());
        }
    }
}
