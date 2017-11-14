//
// Created by Stanislav Olekhnovich on 30/10/2017.
//

#ifndef PATH_PLANNING_CHANGELANETOLEFTSTATE_H
#define PATH_PLANNING_CHANGELANETOLEFTSTATE_H


#include "BehaviorState.h"

class ChangeLaneToLeftState : public BehaviorState
{
public:
    ChangeLaneToLeftState(const HighwayMap &map, int currentLane, double targetSpeed) :
            BehaviorState(map, currentLane, targetSpeed)
    {
        targetLane = currentLane - 1;
    }
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
    std::vector<std::shared_ptr<BehaviorState>> GetNextStates(int currentLane) override;
    std::string GetName() const override { return "ChangeLaneToLeftState"; }
};


#endif //PATH_PLANNING_CHANGELANETOLEFTSTATE_H
