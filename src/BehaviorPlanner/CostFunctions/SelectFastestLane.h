//
// Created by Stanislav Olekhnovich on 07/11/2017.
//

#ifndef PATH_PLANNING_SELECTFASTESTLANE_H
#define PATH_PLANNING_SELECTFASTESTLANE_H

#include "../../PathPlannerInput.h"
#include "../../Settings.h"

class SelectFastestLane
{
public:
    double operator()(const PathPlannerInput& input, int targetLane)
    {
        int closetCarIndex = input.ClosestCarAhead(targetLane);
        if (closetCarIndex != OtherCarNotFound)
        {
            auto& closestCarAhead = input.OtherCars[closetCarIndex];
            if (closestCarAhead.LocationFrenet.S - input.LocationFrenet.S > 60)
                return 0.0;
//            auto res = (MaxSpeedMetersPerSec - closestCarAhead->Speed2DMagnitude()) / MaxSpeedMetersPerSec;
//            std::cout << "SelectFastestLane res: " << res << std::endl;
            return (MaxSpeedMetersPerSec - closestCarAhead.Speed2DMagnitude()) / MaxSpeedMetersPerSec;
        }
        return 0.0;
    }
};

#endif //PATH_PLANNING_SELECTFASTESTLANE_H
