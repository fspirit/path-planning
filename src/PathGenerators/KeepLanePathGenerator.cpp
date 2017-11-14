//
// Created by Stanislav Olekhnovich on 08/11/2017.
//

#include "KeepLanePathGenerator.h"

void KeepLanePathGenerator::AddAnchors(const PathPlannerInput &input, std::vector<CartesianPoint> &anchors) const
{
    int currentLane = input.LocationFrenet.Lane();
    for (auto& i: {30, 60, 90})
    {
        anchors.push_back(map.FrenetToCartesian({input.LocationFrenet.S + i, FrenetPoint::LaneCenterDCoord(currentLane)}));
    }
}


