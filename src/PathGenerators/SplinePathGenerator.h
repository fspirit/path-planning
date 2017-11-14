//
// Created by Stanislav Olekhnovich on 08/11/2017.
//

#ifndef PATH_PLANNING_SPLINEPATHGENERATOR_H
#define PATH_PLANNING_SPLINEPATHGENERATOR_H


#include "../PathPlannerInput.h"
#include "../spline.h"
#include "../HighwayMap.h"

class SplinePathGenerator
{
public:
    SplinePathGenerator(double targetSpeed, const HighwayMap &map) : targetSpeed(targetSpeed), map(map) {}
    std::vector<CartesianPoint> GeneratePath(const PathPlannerInput& input);

protected:
    virtual void AddAnchors(const PathPlannerInput &input, std::vector<CartesianPoint> &anchors) const = 0;
    const HighwayMap& map;
    double targetSpeed;

private:
    std::vector<CartesianPoint> ConvertPointsToLocalSystem(const std::vector<CartesianPoint> &newPathAnchorPoints,
                                                           const CartesianPoint &localReferencePoint) const;
    tk::spline GetSplineFromAnchorPoints(const std::vector<CartesianPoint> &newPathAnchorPoints) const;
    std::vector<CartesianPoint> GenerateNewPointsWithSpline(const tk::spline &newPathSpline, int pointsLeftInCurrentPath) const;

    struct AnchorPointsGenerationResult
    {
        CartesianPoint ReferencePoint;
        std::vector<CartesianPoint> AnchorPoints;

        AnchorPointsGenerationResult(const CartesianPoint &ReferencePoint, const std::vector<CartesianPoint> &AnchorPoints)
                : ReferencePoint(ReferencePoint), AnchorPoints(AnchorPoints) {}
    };

    AnchorPointsGenerationResult GenerateAnchorPoints(const PathPlannerInput& input) const;
};


#endif //PATH_PLANNING_SPLINEPATHGENERATOR_H
