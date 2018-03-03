
#include "Plot.hpp"
#include "Math.hpp"
#include "Log.hpp"
#include "Models.hpp"
#include <sstream>
#include <iomanip>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

namespace slam
{
    static void plotPoseEstimate(const State &state)
    {
        Pose pose = state.getPose().pose;
        double radius = 0.3;
        size_t corners = 16;
        double step = 2 * pi() / corners;

        std::vector<double> x(corners + 1);
        std::vector<double> y(corners + 1);

        for(size_t i = 0; i < corners; ++i)
        {
                x[i] = pose(0) + radius * cos(i * step);
                y[i] = pose(1) + radius * sin(i * step);
        }
        x[corners] = x[0];
        y[corners] = y[0];

        double sx = pose(0);
        double sy = pose(1);
        double tx = pose(0) + radius * cos(pose(2));
        double ty = pose(1) + radius * sin(pose(2));

        plt::plot(x, y, "r-");
        plt::plot({sx, tx}, {sy, ty}, "r-");
    }

    void plotLandmarkEstimates(const State &state)
    {
        size_t n = (state.dim - 3) / 2;
        std::vector<double> x(n);
        std::vector<double> y(n);

        for(size_t i = 0; i < n; ++i)
        {
            StateLandmark lm = state.getLandmark(i);
            x[i] = lm.pos(0);
            y[i] = lm.pos(1);
        }

        plt::plot(x, y, "gx");
    }

    void plotMeasurements(const State& state, const Data &data)
    {
        if(data.observ.empty())
            return;

        Pose pose = state.getPose().pose;

        for(size_t i = 0; i < data.observ.size(); ++i)
        {
            const Measurement &m = data.observ[i].m;
            InvSensorModel sm = calcInvSensorModel(pose, m);

            plt::plot({pose(0), sm.val(0)}, {pose(1), sm.val(1)}, "m-");
        }
    }

    void plotState(const State& state,
        const Data &data,
        const std::vector<Position> &landmarks,
        const std::string &filename)
    {
        plt::clf();

        plt::xlim(-2, 12);
        plt::ylim(-2, 12);

        std::vector<double> lmsX(landmarks.size());
        std::vector<double> lmsY(landmarks.size());

        for(size_t i = 0; i < landmarks.size(); ++i)
        {
            lmsX[i] = landmarks[i](0);
            lmsY[i] = landmarks[i](1);
        }

        plt::plot(lmsX, lmsY, "b*");
        plotPoseEstimate(state);
        plotLandmarkEstimates(state);
        plotMeasurements(state, data);

        plt::save(filename);
    }

    void plotRecords(const std::vector<State>& records,
        const std::vector<Data> &data,
        const std::vector<Position> &landmarks,
        const std::string &prefix)
    {
        for(size_t i = 0; i < records.size(); ++i)
        {
            std::stringstream ss;
            ss << prefix << std::setfill('0') << std::setw(3) << i+1 << ".png";
            plotState(records[i], data[i], landmarks, ss.str());
        }
    }
}
