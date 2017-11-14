//
// Created by Stanislav Olekhnovich on 25/10/2017.
//

const double RegularPriority = 10 * 10;
const double HighPriority = 10 * 10 * 10;

#include <iostream>
#include "CostCalculator.h"

double CostCalculator::CalculateCostForPath(const std::vector<CartesianPoint> &path,
                                            const PathPlannerInput& input, int targetLane)
{
    return avoidCollisionFuction(path, input.OtherCars, map) * HighPriority +
           keepMaxSpeedFunction(path) * RegularPriority +
           selectFastestLaneFunction(input, targetLane) * RegularPriority +
           penaliseLaneChangeFunction(input, targetLane) * RegularPriority;

}

