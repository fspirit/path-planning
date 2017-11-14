//
// Created by Stanislav Olekhnovich on 25/10/2017.
//

#ifndef PATH_PLANNING_KEEPLANESTATE_H
#define PATH_PLANNING_KEEPLANESTATE_H


#include "BehaviorState.h"
#include "../../HighwayMap.h"
#include "../../SimpleSplineBasedPlanner.h"

class KeepLaneState : public BehaviorState
{
public:
    KeepLaneState(const HighwayMap& map, int currentLane, double targetSpeed):
            BehaviorState(map, currentLane, targetSpeed), simpleSplineBasedPlanner(map, currentLane, targetSpeed) {};
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
    std::vector<std::shared_ptr<BehaviorState>> GetNextStates(int currentLane) override;
    std::string GetName() const override { return "KeepLaneState"; }

private:
    SimpleSplineBasedPlanner simpleSplineBasedPlanner;
};


#endif //PATH_PLANNING_KEEPLANESTATE_H
