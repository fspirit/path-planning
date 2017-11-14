//
// Created by Stanislav Olekhnovich on 10/11/2017.
//

#include "ChangeLaneToRightState.h"
#include "../../PathGenerators/RightChangeLanePathGenerator.h"
#include "KeepLaneState.h"

std::vector<CartesianPoint> ChangeLaneToRightState::GeneratePath(PathPlannerInput input)
{
    RightChangeLanePathGenerator pathGenerator = RightChangeLanePathGenerator(targetSpeed, map);
    return pathGenerator.GeneratePath(input);
}

std::vector<std::shared_ptr<BehaviorState>> ChangeLaneToRightState::GetNextStates(int currentLane)
{
    if (currentLane == targetLane)
        return { std::make_shared<KeepLaneState>(map, targetLane, targetSpeed) };
    return { std::make_shared<ChangeLaneToRightState>(map, this->currentLane, targetSpeed) };
}