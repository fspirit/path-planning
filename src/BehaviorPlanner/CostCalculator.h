//
// Created by Stanislav Olekhnovich on 25/10/2017.
//

#ifndef PATH_PLANNING_COSTCALCULATOR_H
#define PATH_PLANNING_COSTCALCULATOR_H

#include <vector>
#include "../CartesianPoint.h"
#include "CostFunctions/AvoidCollision.h"
#include "CostFunctions/KeepMaxSpeed.h"
#include "CostFunctions/SelectFastestLane.h"
#include "../PathPlannerInput.h"
#include "CostFunctions/ChangeLaneOnProperSpeed.h"

class CostCalculator
{
public:
    explicit CostCalculator(const HighwayMap &map): map(map) {};
    double CalculateCostForPath(const std::vector<CartesianPoint>& path, const PathPlannerInput& input, int targetLane);
private:
    const HighwayMap& map;
    KeepMaxSpeed keepMaxSpeedFunction;
    AvoidCollision avoidCollisionFuction;
    SelectFastestLane selectFastestLaneFunction;
    ChangeLaneOnProperSpeed penaliseLaneChangeFunction;

};


#endif //PATH_PLANNING_COSTCALCULATOR_H
