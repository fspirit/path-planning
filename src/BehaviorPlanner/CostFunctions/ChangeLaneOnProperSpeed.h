//
// Created by Stanislav Olekhnovich on 13/11/2017.
//

#ifndef PATH_PLANNING_CHANGELANEONPROPERSPEED_H
#define PATH_PLANNING_CHANGELANEONPROPERSPEED_H


#include "../../OtherCar.h"
#include "../../PathPlannerInput.h"
#include "../../Settings.h"

const double ProperSpeedCost = 0.05;
const double UnproperSpeedCost = 1.0;
const double ChangeLaneAllowedSpeedThreshold = OptimalSpeed * 0.5;

class ChangeLaneOnProperSpeed
{
public:
    double operator()(const PathPlannerInput& input, const int targetLane)
    {
        if (input.LocationFrenet.Lane() != targetLane)
        {
            if (input.Speed < ChangeLaneAllowedSpeedThreshold)
            {
                return UnproperSpeedCost;
            }
            return ProperSpeedCost;
        }
        return 0.0;
    }
};


#endif //PATH_PLANNING_CHANGELANEONPROPERSPEED_H
