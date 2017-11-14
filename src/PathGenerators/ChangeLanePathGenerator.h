//
// Created by Stanislav Olekhnovich on 08/11/2017.
//

#ifndef PATH_PLANNING_CHANGELANEPATHGENERATOR_H
#define PATH_PLANNING_CHANGELANEPATHGENERATOR_H


#include "SplinePathGenerator.h"

class ChangeLanePathGenerator : public SplinePathGenerator
{
public:
    ChangeLanePathGenerator(double targetSpeed, const HighwayMap &map) : SplinePathGenerator(
            targetSpeed, map) {}

protected:
    void AddAnchors(const PathPlannerInput &input, std::vector<CartesianPoint> &anchors) const override;

    virtual int GetTargetLane(int currentLane) const = 0;
    virtual double GetDDeltaSign() const = 0;
    virtual bool AnchorIsCloserThanTargetLaneCenter(int targetLane, const FrenetPoint &anchor) const = 0;
};


#endif //PATH_PLANNING_CHANGELANEPATHGENERATOR_H
