//
// Created by Stanislav Olekhnovich on 25/10/2017.
//

#include "KeepLaneState.h"
#include "ChangeLaneToLeftState.h"
#include "../../Settings.h"
#include "ChangeLaneToRightState.h"

std::vector<CartesianPoint> KeepLaneState::GeneratePath(PathPlannerInput input)
{
    auto path = simpleSplineBasedPlanner.GeneratePath(input);
    targetSpeed = simpleSplineBasedPlanner.GetTargetSpeed();
    return path;
}

std::vector<std::shared_ptr<BehaviorState>> KeepLaneState::GetNextStates(int currentLane)
{
    if (currentLane == 0)
    {
        return { std::make_shared<KeepLaneState>(map, currentLane, targetSpeed),
                 std::make_shared<ChangeLaneToRightState>(map, currentLane, targetSpeed) };
    }
    if (currentLane == NumberOfLines - 1)
    {
        return { std::make_shared<KeepLaneState>(map, currentLane, targetSpeed),
                 std::make_shared<ChangeLaneToLeftState>(map, currentLane, targetSpeed)};
    }
    return {
            std::make_shared<ChangeLaneToLeftState>(map, currentLane, targetSpeed),
             std::make_shared<KeepLaneState>(map, currentLane, targetSpeed),
             std::make_shared<ChangeLaneToRightState>(map, currentLane, targetSpeed) };
}


