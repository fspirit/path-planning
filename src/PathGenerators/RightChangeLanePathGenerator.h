//
// Created by Stanislav Olekhnovich on 10/11/2017.
//

#ifndef PATH_PLANNING_RIGHTCHANGELANEPATHGENERATOR_H
#define PATH_PLANNING_RIGHTCHANGELANEPATHGENERATOR_H

#include "ChangeLanePathGenerator.h"

class RightChangeLanePathGenerator : public ChangeLanePathGenerator
{
public:
    RightChangeLanePathGenerator(double targetSpeed, const HighwayMap &map) : ChangeLanePathGenerator(targetSpeed,
                                                                                                      map) {}
protected:
    int GetTargetLane(int currentLane) const override { return currentLane + 1; }
    double GetDDeltaSign() const override { return 1;  }
    bool AnchorIsCloserThanTargetLaneCenter(int targetLane, const FrenetPoint &anchor) const override
    {
        return anchor.D < FrenetPoint::LaneCenterDCoord(targetLane);
    }
};

#endif //PATH_PLANNING_RIGHTCHANGELANEPATHGENERATOR_H
