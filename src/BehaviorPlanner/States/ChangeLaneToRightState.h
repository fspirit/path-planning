//
// Created by Stanislav Olekhnovich on 10/11/2017.
//

#ifndef PATH_PLANNING_CHANGELANETORIGHTSTATE_H
#define PATH_PLANNING_CHANGELANETORIGHTSTATE_H


#include "BehaviorState.h"

class ChangeLaneToRightState : public BehaviorState
{
public:
    ChangeLaneToRightState(const HighwayMap &map, int currentLane, double targetSpeed) :
            BehaviorState(map, currentLane, targetSpeed)
    {
        targetLane = currentLane + 1;
    }
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
    std::vector<std::shared_ptr<BehaviorState>> GetNextStates(int currentLane) override;
    std::string GetName() const override { return "ChangeLaneToRightState"; }
};


#endif //PATH_PLANNING_CHANGELANETORIGHTSTATE_H
