//
// Created by Stanislav Olekhnovich on 30/10/2017.
//

#include "ChangeLaneToLeftState.h"
#include "../../PathGenerators/LeftChangeLanePathGenerator.h"
#include "KeepLaneState.h"

std::vector<CartesianPoint> ChangeLaneToLeftState::GeneratePath(PathPlannerInput input)
{
    LeftChangeLanePathGenerator pathGenerator = LeftChangeLanePathGenerator(targetSpeed, map);
    return pathGenerator.GeneratePath(input);
}

std::vector<std::shared_ptr<BehaviorState>> ChangeLaneToLeftState::GetNextStates(int currentLane)
{
    if (currentLane == targetLane)
        return { std::make_shared<KeepLaneState>(map, currentLane, targetSpeed) };
    return { std::make_shared<ChangeLaneToLeftState>(map, this->currentLane, targetSpeed) };
}
