//
// Created by Stanislav Olekhnovich on 08/11/2017.
//

#ifndef PATH_PLANNING_KEEPLANEPATHGENERATOR_H
#define PATH_PLANNING_KEEPLANEPATHGENERATOR_H


#include <vector>
#include "../CartesianPoint.h"
#include "../PathPlannerInput.h"
#include "SplinePathGenerator.h"

class KeepLanePathGenerator : public SplinePathGenerator
{
public:
    KeepLanePathGenerator(double targetSpeed, const HighwayMap &map) : SplinePathGenerator(
            targetSpeed, map) {}
    std::vector<CartesianPoint> GeneratePath(const PathPlannerInput& input);

protected:
    void AddAnchors(const PathPlannerInput &input, std::vector<CartesianPoint> &anchors) const override;
};


#endif //PATH_PLANNING_KEEPLANEPATHGENERATOR_H
