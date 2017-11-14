//
// Created by Stanislav Olekhnovich on 12/10/2017.
//

#ifndef PATH_PLANNING_PATHPLANNERINPUT_H
#define PATH_PLANNING_PATHPLANNERINPUT_H

#include <vector>

#include "CartesianPoint.h"
#include "FrenetPoint.h"
#include "OtherCar.h"

const int OtherCarNotFound = -1;

struct PathPlannerInput
{
    CartesianPoint LocationCartesian;
    FrenetPoint LocationFrenet;
    FrenetPoint PathEndpointFrenet;

    double Speed;

    std::vector<CartesianPoint> Path;

    std::vector<double> PreviousPathX;
    std::vector<double> PreviousPathY;

    std::vector<OtherCar> OtherCars;

    int ClosestCarAhead(int lane) const
    {
        int result = OtherCarNotFound;
        double minSDiff = std::numeric_limits<double>::max();
        for (int i = 0; i < OtherCars.size(); i++)
        {
            auto& otherCar = OtherCars[i];
            if (otherCar.IsInLane(lane))
            {
                auto sDiff = otherCar.LocationFrenet.S - LocationFrenet.S;
                if (sDiff > 0 && sDiff < minSDiff)
                {
                    result = i;
                    minSDiff = sDiff;
                }
            }
        }
        return result;
    }

    int ClosestCarBehind(int lane)
    {
        int result = OtherCarNotFound;
        double minSDiff = std::numeric_limits<double>::max();
        for (int i = 0; i < OtherCars.size(); i++)
        {
            auto& otherCar = OtherCars[i];
            if (otherCar.IsInLane(lane))
            {
                auto sDiff = LocationFrenet.S - otherCar.LocationFrenet.S;
                if (sDiff > 0 && sDiff < minSDiff)
                {
                    result = i;
                    minSDiff = sDiff;
                }
            }
        }
        return result;
    }
};


#endif //PATH_PLANNING_PATHPLANNERINPUT_H
