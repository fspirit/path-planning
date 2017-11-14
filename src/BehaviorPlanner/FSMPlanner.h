//
// Created by Stanislav Olekhnovich on 25/10/2017.
//

#ifndef PATH_PLANNING_FSMPLANNER_H
#define PATH_PLANNING_FSMPLANNER_H

#include <memory>

#include "../PathPlanner.h"
#include "States/BehaviorState.h"
#include "CostCalculator.h"
#include "States/KeepLaneState.h"

class FSMPlanner : public PathPlanner
{
public:
    explicit FSMPlanner(const HighwayMap& map, int startingLane, CostCalculator costCalculator):
            PathPlanner(map, startingLane),
            state(std::make_shared<KeepLaneState>(map, startingLane, StartingTargetSpeed)),
            costCalculator(costCalculator) {};
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
private:
    CostCalculator costCalculator;
    std::shared_ptr<BehaviorState> state;
};


#endif //PATH_PLANNING_FSMPLANNER_H
