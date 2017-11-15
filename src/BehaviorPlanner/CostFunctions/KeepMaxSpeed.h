//
// Created by Stanislav Olekhnovich on 30/10/2017.
//

#ifndef PATH_PLANNING_KEEPMAXSPEED_H
#define PATH_PLANNING_KEEPMAXSPEED_H

#include <vector>

#include "../../CartesianPoint.h"
#include "../../Settings.h"
#include "../../spline.h"

const double StopCost = 0.8;

const double OptimalSpeedInMs = MphToMs(OptimalSpeed);

class KeepMaxSpeed
{
public:
    KeepMaxSpeed()
    {
        firstInterval.set_points({ 0.0, OptimalSpeedInMs / 2.0, OptimalSpeedInMs }, { StopCost, StopCost / 2.0, 0.0 });
        secondInterval.set_points({ OptimalSpeedInMs, OptimalSpeedInMs + (MaxSpeedMetersPerSec - OptimalSpeedInMs) / 2.0, MaxSpeedMetersPerSec }, { 0.0, 0.5, 1.0 });
    }

    double operator()(const std::vector<CartesianPoint>& path)
    {
        double xAxisSpeed = (path.back().X - path.front().X) / (path.size() * SimulatorRunloopPeriod);
        double yAxisSpeed = (path.back().Y - path.front().Y) / (path.size() * SimulatorRunloopPeriod);

        double speed = sqrt(xAxisSpeed * xAxisSpeed + yAxisSpeed * yAxisSpeed);

        if (speed <= 0.0)
        {
            return StopCost;
        }
        else if (speed > 0.0 && speed <= OptimalSpeedInMs)
        {
            return firstInterval(speed);
        }
        else if (speed > OptimalSpeedInMs && speed < MaxSpeedMetersPerSec)
        {
            return secondInterval(speed);
        }
        else
        {
            return 1.0;
        }
    }

private:
    tk::spline firstInterval;
    tk::spline secondInterval;
};


#endif //PATH_PLANNING_KEEPMAXSPEED_H
