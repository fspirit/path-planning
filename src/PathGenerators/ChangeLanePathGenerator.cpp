//
// Created by Stanislav Olekhnovich on 08/11/2017.
//

#include <iostream>
#include "ChangeLanePathGenerator.h"

void ChangeLanePathGenerator::AddAnchors(const PathPlannerInput &input, std::vector<CartesianPoint> &anchors) const
{
//    std::cout << "----" << std::endl;
//    for (auto& p: anchors)
//    {
//        std::cout << p.X << ", " << p.Y << ", " << p.Theta << std::endl;
//    }
    int currentLane = input.LocationFrenet.Lane();
    int targetLane = GetTargetLane(currentLane);

    std::vector<FrenetPoint> pp;

    const double ds = 10.0;
    double dd = 1.0 * GetDDeltaSign();

    FrenetPoint furthestAnchor;
    if (input.Path.size() == 0)
    {
        furthestAnchor = { input.LocationFrenet.S + ds,  input.LocationFrenet.D + dd };
    }
    else
    {
        furthestAnchor = { input.PathEndpointFrenet.S + ds,  input.PathEndpointFrenet.D + dd };
    }
    while (AnchorIsCloserThanTargetLaneCenter(targetLane, furthestAnchor))
    {
        anchors.push_back(map.FrenetToCartesian(furthestAnchor));
        pp.push_back(furthestAnchor);
        furthestAnchor = { furthestAnchor.S + ds,  furthestAnchor.D + dd };
    }

    for (auto& s: {furthestAnchor.S + 20, furthestAnchor.S + 40, furthestAnchor.S + 60})
    {
        pp.push_back({s, FrenetPoint::LaneCenterDCoord(targetLane)});
        anchors.push_back(map.FrenetToCartesian({s, FrenetPoint::LaneCenterDCoord(targetLane)}));
    }

//    std::cout << "----" << std::endl;
//    for (auto& fp: pp)
//    {
//        std::cout << fp.S << ", " << fp.D << ", " << fp.Lane() << std::endl;
//    }
//    std::cout << "----" << std::endl;
//    for (auto& p: anchors)
//    {
//        std::cout << p.X << ", " << p.Y << ", " << p.Theta << std::endl;
//    }


}


