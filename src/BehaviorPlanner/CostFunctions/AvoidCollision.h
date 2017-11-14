//
// Created by Stanislav Olekhnovich on 06/11/2017.
//

#ifndef PATH_PLANNING_AVOIDCOLLISION_H
#define PATH_PLANNING_AVOIDCOLLISION_H

#include <map>

#include "../../CartesianPoint.h"
#include "../../OtherCar.h"
#include "../../HighwayMap.h"
#include "../../Settings.h"

const double CriticalSDistance = 15.0;

class AvoidCollision
{
public:
    double operator()(const std::vector<CartesianPoint>& path,
                      const std::vector<OtherCar>& otherCars,
                      const HighwayMap& map)
    {
        // We assume other cars are not changing lanes and keeping const speed
        for (auto& otherCar : otherCars)
        {
            if (otherCar.LocationFrenet.D < .0 || otherCar.LocationFrenet.D > 12.0)
                continue;

            for (int j = 0; j < path.size(); j++)
            {
                auto frenetPoint = map.CartesianToFrenet(path[j]);
                FrenetPoint predictedOtherCarPosition = {
                        otherCar.LocationFrenet.S + otherCar.Speed2DMagnitude() * SimulatorRunloopPeriod * j,
                        otherCar.LocationFrenet.D};
                if (frenetPoint.Lane() == predictedOtherCarPosition.Lane() &&
                    fabs(frenetPoint.S - predictedOtherCarPosition.S) < CriticalSDistance)
                {
                    return 1.0;
                }
            }
        }
        return .0;
    }
};

#endif //PATH_PLANNING_AVOIDCOLLISION_H
