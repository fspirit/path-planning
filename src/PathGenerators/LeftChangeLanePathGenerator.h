//
// Created by Stanislav Olekhnovich on 10/11/2017.
//

#ifndef PATH_PLANNING_LEFTCHANGELANEPATHGENERATOR_H
#define PATH_PLANNING_LEFTCHANGELANEPATHGENERATOR_H

#include "ChangeLanePathGenerator.h"

class LeftChangeLanePathGenerator : public ChangeLanePathGenerator
{
public:
    LeftChangeLanePathGenerator(double targetSpeed, const HighwayMap &map) : ChangeLanePathGenerator(targetSpeed,
                                                                                                     map) {}
protected:
    int GetTargetLane(int currentLane) const override { return currentLane - 1;}
    double GetDDeltaSign() const override { return -1; }
    bool AnchorIsCloserThanTargetLaneCenter(int targetLane, const FrenetPoint &anchor) const override
    {
        return anchor.D > FrenetPoint::LaneCenterDCoord(targetLane);
    }
};

#endif //PATH_PLANNING_LEFTCHANGELANEPATHGENERATOR_H
