//
// Created by Stanislav Olekhnovich on 25/10/2017.
//

#include <iostream>
#include "FSMPlanner.h"

std::vector<CartesianPoint> FSMPlanner::GeneratePath(PathPlannerInput input)
{
    auto possibleNewStates = state->GetNextStates(input.LocationFrenet.Lane());

    std::shared_ptr<BehaviorState> newState;
    std::vector<CartesianPoint> selectedPath;
    double minCost = std::numeric_limits<double>::max();

    for (auto& possibleNextState : possibleNewStates)
    {
        auto statePath = possibleNextState->GeneratePath(input);
        auto cost = costCalculator.CalculateCostForPath(statePath, input, possibleNextState->GetTargetLane());
        std::cout << possibleNextState->GetName() << ": " << cost << std::endl;

        if (cost < minCost)
        {
            minCost = cost;
            newState = possibleNextState;
            selectedPath = statePath;
        }
    }

    state = newState;

    std::cout << "Selected state: " << state->GetName() << std::endl;

    return selectedPath;
}
