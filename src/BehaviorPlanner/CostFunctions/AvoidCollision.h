//
// Created by Stanislav Olekhnovich on 06/11/2017.
//

#ifndef PATH_PLANNING_AVOIDCOLLISION_H
#define PATH_PLANNING_AVOIDCOLLISION_H

#include <map>
#include <cmath>

#include "../../CartesianPoint.h"
#include "../../OtherCar.h"
#include "../../HighwayMap.h"
#include "../../Settings.h"
#include "../../PathPlannerInput.h"

const double CriticalSDistance = 20.0;

class AvoidCollision
{
public:
    double operator()(const std::vector<CartesianPoint>& path,
                      const PathPlannerInput& input,
                      const HighwayMap& map,
                      int targetLane)
    {
        if (input.LocationFrenet.Lane() == targetLane)
            return 0.0;

        // We assume other cars are not changing lanes and keeping const speed
        for (auto& otherCar : input.OtherCars)
        {
            if (otherCar.LocationFrenet.Lane() != targetLane)
                continue;

            for (int j = 0; j < path.size(); j++)
            {
                CartesianPoint predictedOtherCarPosition = {
                        otherCar.LocationCartesian.X + otherCar.XAxisSpeed * SimulatorRunloopPeriod * j,
                        otherCar.LocationCartesian.Y + otherCar.YAxisSpeed * SimulatorRunloopPeriod * j};
                if (map.EuclidDistance(path[j], predictedOtherCarPosition) < CriticalSDistance)
                {
                    return 1.0;
                }
            }
        }
        return .0;
    }
};

#endif //PATH_PLANNING_AVOIDCOLLISION_H
