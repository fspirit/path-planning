//
// Created by Stanislav Olekhnovich on 03/11/2017.
//

#ifndef PATH_PLANNING_SPLINEBASEDLEFTCHANGECAPABLEPLANNER_H
#define PATH_PLANNING_SPLINEBASEDLEFTCHANGECAPABLEPLANNER_H

#include "SimpleSplineBasedPlanner.h"
#include "PathGenerators/KeepLanePathGenerator.h"
#include "PathGenerators/RightChangeLanePathGenerator.h"
#include "PathGenerators/LeftChangeLanePathGenerator.h"

class SplineBasedLeftChangeCapablePlanner : public SimpleSplineBasedPlanner
{
public:
    SplineBasedLeftChangeCapablePlanner(const HighwayMap &map, int startingLane, double targetSpeed)
            : SimpleSplineBasedPlanner(map, startingLane, targetSpeed) {}

protected:
    virtual std::shared_ptr<SplinePathGenerator> CreatePathGenerator(bool otherCarTooCloseAhead) override
    {
        if (otherCarTooCloseAhead && currentLane != 0 && currentLane != (NumberOfLines - 1))
            return std::make_shared<RightChangeLanePathGenerator>(targetSpeed, map);
        return std::make_shared<KeepLanePathGenerator>(targetSpeed, map);
    }
};


#endif //PATH_PLANNING_SPLINEBASEDLEFTCHANGECAPABLEPLANNER_H
